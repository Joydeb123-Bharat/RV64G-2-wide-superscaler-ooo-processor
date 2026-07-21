`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 18.07.2026 14:34:51
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: dispatch_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Rename and dispatch stage. Reads the RAT to translate architectural
//              source registers to physical tags, pops new physical destination tags
//              from the Free List, allocates LDQ/STQ entries in the LSQ, writes
//              ROB entries, and dispatches to the Issue Queue. Handles intra-dispatch
//              forwarding (slot 0 write visible to slot 1 read). Asserts dispatch_stall
//              when the ROB, IQ, LDQ, STQ, or Free List cannot accept new instructions.
//              Saves a GHR checkpoint in the branch predictor for every branch dispatched.
//
// Dependencies: rat, free_list, rob, issue_queue, lsq, prf, branch_predictor
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   dispatch_stall is the primary back-pressure signal fed back to fetch and decode.
//   dec_is_load/store are used (not indexed) when driving lsq_dispatch — see ooo_core.sv.
//
//////////////////////////////////////////////////////////////////////////////////

module dispatch_unit #(
    parameter DATA_W      = 64,   // data and PC width
    parameter ISSUE_WIDTH = 2,    // instructions dispatched per cycle
    parameter PHYS_TAG_W  = 7,    // physical register tag width
    parameter ROB_IDX_W   = 6,    // ROB index width
    parameter GHR_W       = 12    // branch predictor GHR width
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // flush interface
    input  logic                      flush,             // global flush from ROB

    // decoded input from decode unit
    input  logic [ISSUE_WIDTH-1:0]    dec_valid,                           // valid decoded slot
    input  logic [4:0]                dec_rs1        [0:ISSUE_WIDTH-1],    // architectural src1
    input  logic [4:0]                dec_rs2        [0:ISSUE_WIDTH-1],    // architectural src2
    input  logic [4:0]                dec_rd         [0:ISSUE_WIDTH-1],    // architectural dst
    input  logic [DATA_W-1:0]         dec_imm        [0:ISSUE_WIDTH-1],    // sign-extended immediate
    input  logic [DATA_W-1:0]         dec_pc         [0:ISSUE_WIDTH-1],    // instruction PC
    input  logic [3:0]                dec_alu_op     [0:ISSUE_WIDTH-1],    // ALU operation code
    input  logic [2:0]                dec_funct3     [0:ISSUE_WIDTH-1],    // funct3 for branch MULDIV
    input  logic [1:0]                dec_unit_sel   [0:ISSUE_WIDTH-1],    // target execution unit
    input  logic [1:0]                dec_mem_size   [0:ISSUE_WIDTH-1],    // memory access size
    input  logic [ISSUE_WIDTH-1:0]    dec_uses_imm,                        // uses immediate flag
    input  logic [ISSUE_WIDTH-1:0]    dec_is_load,                         // load instruction
    input  logic [ISSUE_WIDTH-1:0]    dec_is_store,                        // store instruction
    input  logic [ISSUE_WIDTH-1:0]    dec_is_branch,                       // conditional branch
    input  logic [ISSUE_WIDTH-1:0]    dec_is_jalr,                         // JALR instruction
    input  logic [ISSUE_WIDTH-1:0]    dec_is_jal,                          // JAL instruction
    input  logic [ISSUE_WIDTH-1:0]    dec_is_mul,                          // MUL instruction
    input  logic [ISSUE_WIDTH-1:0]    dec_is_div,                          // DIV instruction
    input  logic [ISSUE_WIDTH-1:0]    dec_sign_ext,                        // sign extend load
    input  logic [ISSUE_WIDTH-1:0]    dec_has_rd,                          // writes destination
    input  logic [ISSUE_WIDTH-1:0]    dec_is_word,                         // word operation
    input  logic [ISSUE_WIDTH-1:0]    dec_illegal,                         // illegal instruction

    // RAT interface
    output logic [4:0]                rat_rs1_arch   [0:ISSUE_WIDTH-1],    // src1 arch reg to RAT
    output logic [4:0]                rat_rs2_arch   [0:ISSUE_WIDTH-1],    // src2 arch reg to RAT
    input  logic [PHYS_TAG_W-1:0]     rat_rs1_phys   [0:ISSUE_WIDTH-1],    // src1 phys tag from RAT
    input  logic [PHYS_TAG_W-1:0]     rat_rs2_phys   [0:ISSUE_WIDTH-1],    // src2 phys tag from RAT
    output logic [ISSUE_WIDTH-1:0]    rat_wr_en,                           // RAT write enable
    output logic [4:0]                rat_rd_arch    [0:ISSUE_WIDTH-1],    // dst arch reg to RAT
    output logic [PHYS_TAG_W-1:0]     rat_rd_phys    [0:ISSUE_WIDTH-1],    // new phys tag to RAT
    input  logic [PHYS_TAG_W-1:0]     rat_stale_phys [0:ISSUE_WIDTH-1],    // stale phys from RAT

    // Free List interface
    output logic [ISSUE_WIDTH-1:0]    fl_pop_req,                          // pop request per slot
    input  logic [PHYS_TAG_W-1:0]     fl_pop_tag     [0:ISSUE_WIDTH-1],    // popped physical tag
    input  logic [ISSUE_WIDTH-1:0]    fl_pop_valid,                        // pop tag valid
    input  logic                      fl_stall,                            // free list nearly empty
    output logic                      fl_ckpt_en,                          // checkpoint on branch
    output logic [ROB_IDX_W-1:0]      fl_ckpt_idx,                         // ROB idx for checkpoint
    output logic [1:0]                fl_ckpt_offset,                      // pops already committed (1 for JAL/JALR, 0 for branch)

    // ROB interface
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_en,                     // dispatch enable
    output logic [DATA_W-1:0]         rob_dispatch_pc       [0:ISSUE_WIDTH-1], // PC per slot
    output logic [4:0]                rob_dispatch_arch_rd  [0:ISSUE_WIDTH-1], // arch rd
    output logic [PHYS_TAG_W-1:0]     rob_dispatch_phys_rd  [0:ISSUE_WIDTH-1], // phys rd P_new
    output logic [PHYS_TAG_W-1:0]     rob_dispatch_stale    [0:ISSUE_WIDTH-1], // stale tag P_old
    output logic [4:0]                rob_dispatch_lsq_idx  [0:ISSUE_WIDTH-1], // LSQ slot
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_store,                // store flag
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_load,                 // load flag
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_branch,               // branch flag
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_jal,                  // JAL flag
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_jalr,                 // JALR flag
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_has_rd,                  // has destination
    output logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_illegal,              // illegal instr flag
    input  logic [5:0]                rob_idx        [0:ISSUE_WIDTH-1],     // assigned ROB slots
    input  logic                      rob_full,                             // ROB has no space

    // IQ interface
    output logic [ISSUE_WIDTH-1:0]    iq_dispatch_en,                       // dispatch enable
    output logic [PHYS_TAG_W-1:0]     iq_src1        [0:ISSUE_WIDTH-1],     // physical src1
    output logic [PHYS_TAG_W-1:0]     iq_src2        [0:ISSUE_WIDTH-1],     // physical src2
    output logic [PHYS_TAG_W-1:0]     iq_dst         [0:ISSUE_WIDTH-1],     // physical dst
    output logic [ISSUE_WIDTH-1:0]    iq_src1_rdy,                          // src1 ready at dispatch
    output logic [ISSUE_WIDTH-1:0]    iq_src2_rdy,                          // src2 ready at dispatch
    output logic [ROB_IDX_W-1:0]      iq_rob_idx     [0:ISSUE_WIDTH-1],     // ROB slot
    output logic [3:0]                iq_alu_op      [0:ISSUE_WIDTH-1],     // ALU op code
    output logic [DATA_W-1:0]         iq_imm         [0:ISSUE_WIDTH-1],     // immediate value
    output logic [DATA_W-1:0]         iq_pc          [0:ISSUE_WIDTH-1],     // instruction PC
    output logic [2:0]                iq_funct3      [0:ISSUE_WIDTH-1],     // funct3 field
    output logic [1:0]                iq_unit_sel    [0:ISSUE_WIDTH-1],     // target unit
    output logic [ISSUE_WIDTH-1:0]    iq_uses_imm,                          // uses immediate
    output logic [ISSUE_WIDTH-1:0]    iq_is_load,                           // load flag
    output logic [ISSUE_WIDTH-1:0]    iq_is_store,                          // store flag
    output logic [ISSUE_WIDTH-1:0]    iq_is_branch,                         // conditional branch
    output logic [ISSUE_WIDTH-1:0]    iq_is_jal,                            // JAL flag
    output logic [ISSUE_WIDTH-1:0]    iq_is_jalr,                           // JALR flag
    output logic [ISSUE_WIDTH-1:0]    iq_is_mul,                            // MUL flag
    output logic [ISSUE_WIDTH-1:0]    iq_is_div,                            // DIV flag
    output logic [ISSUE_WIDTH-1:0]    iq_is_word,                           // word operation
    output logic [ISSUE_WIDTH-1:0]    iq_predicted_taken,                   // branch prediction
    output logic [DATA_W-1:0]         iq_predicted_target [0:ISSUE_WIDTH-1],// predicted target
    output logic [GHR_W-1:0]          iq_ghr_snapshot[0:ISSUE_WIDTH-1],     // GHR at dispatch
    input  logic                      iq_full,                              // IQ has no space

    // LSQ interface
    output logic [ISSUE_WIDTH-1:0]    lsq_dispatch_en,                      // dispatch enable
    output logic [ISSUE_WIDTH-1:0]    lsq_dispatch_is_load,                 // load flag
    output logic [ISSUE_WIDTH-1:0]    lsq_dispatch_is_store,                // store flag
    output logic [ROB_IDX_W-1:0]      lsq_dispatch_rob_idx [0:ISSUE_WIDTH-1],// ROB idx
    output logic [PHYS_TAG_W-1:0]     lsq_dispatch_phys_dst[0:ISSUE_WIDTH-1],// phys dst
    output logic [1:0]                lsq_dispatch_mem_size[0:ISSUE_WIDTH-1],// access size
    output logic [ISSUE_WIDTH-1:0]    lsq_dispatch_sign_ext,                // sign extend
    input  logic [4:0]                lsq_ldq_idx    [0:ISSUE_WIDTH-1],     // allocated LDQ slot
    input  logic [4:0]                lsq_stq_idx    [0:ISSUE_WIDTH-1],     // allocated STQ slot
    input  logic                      ldq_full,                             // LDQ no space
    input  logic                      stq_full,                             // STQ no space

    // PRF interface - valid bit check for initial src_rdy
    output logic [PHYS_TAG_W-1:0]     prf_disp_tag   [0:ISSUE_WIDTH*2-1],   // src tags to PRF
    input  logic                      prf_disp_valid [0:ISSUE_WIDTH*2-1],   // valid bits from PRF

    // branch predictor interface
    input  logic [GHR_W-1:0]          bp_current_ghr,                       // current GHR value
    input  logic [ISSUE_WIDTH-1:0]    bp_predicted_taken,                   // per slot prediction
    input  logic [DATA_W-1:0]         bp_predicted_target[0:ISSUE_WIDTH-1], // predicted target
    output logic                      bp_ckpt_en,                           // checkpoint GHR
    output logic [ROB_IDX_W-1:0]      bp_ckpt_rob_idx,                      // ROB idx for GHR ckpt

    // stall output
    output logic                      dispatch_stall                        // stall front-end
);

    // local parameters
    localparam UNIT_ALU    = 2'b00;  // ALU execution unit selector
    localparam UNIT_MULDIV = 2'b01;  // MULDIV execution unit selector
    localparam UNIT_LDST   = 2'b10;  // load store unit selector

    // renamed physical tags after bypass resolution
    logic [PHYS_TAG_W-1:0]  phys_src1  [0:ISSUE_WIDTH-1];  // final src1 tag after bypass
    logic [PHYS_TAG_W-1:0]  phys_src2  [0:ISSUE_WIDTH-1];  // final src2 tag after bypass
    logic [PHYS_TAG_W-1:0]  phys_dst   [0:ISSUE_WIDTH-1];  // fresh tag from Free List
    logic [PHYS_TAG_W-1:0]  phys_stale [0:ISSUE_WIDTH-1];  // stale tag after bypass

    // intra-dispatch bypass detection flags
    logic                   bypass_src1  [0:ISSUE_WIDTH-1]; // src1 matches earlier slot dst
    logic                   bypass_src2  [0:ISSUE_WIDTH-1]; // src2 matches earlier slot dst
    logic                   bypass_stale [0:ISSUE_WIDTH-1]; // dst matches earlier slot dst
    logic                   slot0_writes;
    
    // stall qualification signals
    logic                   need_fl;         // at least one slot needs a physical register
    logic                   need_ldq;        // at least one slot is a load
    logic                   need_stq;        // at least one slot is a store
    logic                   resource_stall;  // any resource unavailable this cycle

    // per slot dispatch gate
    logic [ISSUE_WIDTH-1:0] dispatch_ok;     // slot is valid and all resources available

    // LSQ index per slot after load vs store selection
    logic [4:0]             lsq_idx [0:ISSUE_WIDTH-1];  // selected LDQ or STQ index per slot

    // initial src_rdy after PRF check and bypass
    logic [ISSUE_WIDTH-1:0] src1_rdy;  // src1 is ready at dispatch time
    logic [ISSUE_WIDTH-1:0] src2_rdy;  // src2 is ready at dispatch time
    
    always_comb
    begin
        // internal signals
        need_fl          = 1'b0;
        need_ldq         = 1'b0;
        need_stq         = 1'b0;
        resource_stall   = 1'b0;
        dispatch_stall   = 1'b0;
        fl_ckpt_en       = 1'b0;
        fl_ckpt_idx      = '0;
        fl_ckpt_offset   = 2'd0;
        bp_ckpt_en       = 1'b0;
        bp_ckpt_rob_idx  = '0;
    
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            dispatch_ok[i]   = 1'b0;
            phys_src1[i]     = '0;
            phys_src2[i]     = '0;
            phys_dst[i]      = '0;
            phys_stale[i]    = '0;
            bypass_src1[i]   = 1'b0;
            bypass_src2[i]   = 1'b0;
            bypass_stale[i]  = 1'b0;
            lsq_idx[i]       = '0;
            src1_rdy[i]      = 1'b0;
            src2_rdy[i]      = 1'b0;
        end
    
        // output ports - RAT
        rat_wr_en        = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            rat_rs1_arch[i] = '0;
            rat_rs2_arch[i] = '0;
            rat_rd_arch[i]  = '0;
            rat_rd_phys[i]  = '0;
        end
    
        // output ports - Free List
        fl_pop_req       = '0;
    
        // output ports - ROB
        rob_dispatch_en          = '0;
        rob_dispatch_is_store    = '0;
        rob_dispatch_is_load     = '0;
        rob_dispatch_is_branch   = '0;
        rob_dispatch_is_jal      = '0;
        rob_dispatch_is_jalr     = '0;
        rob_dispatch_has_rd      = '0;
        rob_dispatch_is_illegal  = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            rob_dispatch_pc[i]      = '0;
            rob_dispatch_arch_rd[i] = '0;
            rob_dispatch_phys_rd[i] = '0;
            rob_dispatch_stale[i]   = '0;
            rob_dispatch_lsq_idx[i] = '0;
        end
    
        // output ports - IQ
        iq_dispatch_en   = '0;
        iq_src1_rdy      = '0;
        iq_src2_rdy      = '0;
        iq_uses_imm      = '0;
        iq_is_load       = '0;
        iq_is_store      = '0;
        iq_is_branch     = '0;
        iq_is_jal        = '0;
        iq_is_jalr       = '0;
        iq_is_mul        = '0;
        iq_is_div        = '0;
        iq_is_word       = '0;
        iq_predicted_taken = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            iq_src1[i]             = '0;
            iq_src2[i]             = '0;
            iq_dst[i]              = '0;
            iq_rob_idx[i]          = '0;
            iq_alu_op[i]           = '0;
            iq_imm[i]              = '0;
            iq_pc[i]               = '0;
            iq_funct3[i]           = '0;
            iq_unit_sel[i]         = '0;
            iq_predicted_target[i] = '0;
            iq_ghr_snapshot[i]     = '0;
        end
    
        // output ports - LSQ
        lsq_dispatch_en      = '0;
        lsq_dispatch_is_load = '0;
        lsq_dispatch_is_store= '0;
        lsq_dispatch_sign_ext= '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            lsq_dispatch_rob_idx[i]  = '0;
            lsq_dispatch_phys_dst[i] = '0;
            lsq_dispatch_mem_size[i] = '0;
        end
    
        // output ports - PRF dispatch valid check
        for(int i = 0; i < ISSUE_WIDTH*2; i++)
            prf_disp_tag[i] = '0;
    
        // output - stall
        dispatch_stall = 1'b0;
        
        // RAT Source Lookup
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            rat_rs1_arch[i] = dec_rs1[i];
            rat_rs2_arch[i] = dec_rs2[i];
            rat_rd_arch[i]  = dec_rd[i];
            phys_src1[i]    = rat_rs1_phys[i];
            phys_src2[i]    = rat_rs2_phys[i];
            phys_stale[i]   = rat_stale_phys[i];
        end
        
        // Resources Availablity
        need_fl = |(dec_valid & dec_has_rd);
        need_ldq= |(dec_valid & dec_is_load);
        need_stq= |(dec_valid & dec_is_store);
        
        // Stall computation
        resource_stall = rob_full | iq_full | (fl_stall & need_fl) | (ldq_full & need_ldq) | (stq_full & need_stq);
        dispatch_stall = resource_stall & (|dec_valid);
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            dispatch_ok[i] = dec_valid[i] & !resource_stall & !flush;
        end
        
        // Free List Pop request
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            fl_pop_req[i] = dec_valid[i] & dec_has_rd[i];
            phys_dst[i]   = (dec_valid[i] & dec_has_rd[i] & fl_pop_valid[i]) ? fl_pop_tag[i] : '0;
        end
        
        phys_src1[0] = rat_rs1_phys[0];
        phys_src2[0] = rat_rs2_phys[0];
        phys_stale[0]= rat_stale_phys[0];
        
        // Intra Dispatch Bypass
        slot0_writes    = dispatch_ok[0] & dec_has_rd[0] & (dec_rd[0] != '0);
        bypass_src1[1]  = slot0_writes & dec_has_rd[1] & (dec_rs1[1] == dec_rd[0]) & (dec_rs1[1] != '0);
        bypass_src2[1]  = slot0_writes & dec_has_rd[1] & (dec_rs2[1] == dec_rd[0]) & (dec_rs2[1] != '0);
        if(bypass_src1[1])
        begin
            phys_src1[1] = phys_dst[0];
            src1_rdy[1]  = 1'b0;
        end
        else
        begin
            phys_src1[1] = rat_rs1_phys[1];
            src1_rdy[1]  = (dec_rs1[1] == '0) | prf_disp_valid[2];
        end
        if(bypass_src2[1])
        begin
            phys_src2[1] = phys_dst[0];
            src2_rdy[1]  = 1'b0;
        end
        else
        begin
            phys_src2[1] = rat_rs2_phys[1];
            src2_rdy[1]  = (dec_rs2[1] == '0) | prf_disp_valid[3];
        end
        bypass_stale[1] = slot0_writes & (dec_rd[1] == dec_rd[0]);
        phys_stale[1]   = bypass_stale[1] ? phys_dst[0] : rat_stale_phys[1];

        src1_rdy[0]  = (dec_rs1[0] == '0) | prf_disp_valid[0];
        src2_rdy[0]  = (dec_rs2[0] == '0) | prf_disp_valid[1];
         
        // PRF tag output port
        prf_disp_tag[0] = phys_src1[0];
        prf_disp_tag[1] = phys_src2[0];
        prf_disp_tag[2] = phys_src1[1];
        prf_disp_tag[3] = phys_src2[1];
        
        // LSQ Index Selection
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            if(dec_valid[i] && dec_is_load[i])
                lsq_idx[i] = lsq_ldq_idx[i];
            else if(dec_valid[i] && dec_is_store[i])
                lsq_idx[i] = lsq_stq_idx[i];
        end
        
        // Branch Checkpoint
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            if(dispatch_ok[i] & (dec_is_branch[i] | dec_is_jal[i] | dec_is_jalr[i]))
            begin
                bp_ckpt_en      = 1;
                bp_ckpt_rob_idx = rob_idx[i];
                fl_ckpt_en      = 1;
                fl_ckpt_idx     = rob_idx[i];
                // fl_ckpt_offset: how many pops (in this dispatch) belong to committed
                // instructions up to and including this slot. Wrong-path starts after.
                // JAL/JALR have dec_has_rd=1, regular branches have dec_has_rd=0.
                fl_ckpt_offset  = 2'd0;
                for (int j = 0; j <= i; j++) begin
                    if (dispatch_ok[j] && dec_has_rd[j])
                        fl_ckpt_offset += 2'd1;
                end
            end
        end
        
        // Driving All modules
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            if(dispatch_ok[i])
            begin
                rat_wr_en[i]    = dec_has_rd[i] & (dec_rd[i] != 5'd0);
                rat_rd_arch[i]  = dec_rd[i];
                rat_rd_phys[i]  = phys_dst[i];
                
                rob_dispatch_en[i]          = 1'b1;
                rob_dispatch_pc[i]          = dec_pc[i];
                rob_dispatch_arch_rd[i]     = dec_rd[i];
                rob_dispatch_phys_rd[i]     = phys_dst[i];
                rob_dispatch_stale[i]       = phys_stale[i];
                rob_dispatch_lsq_idx[i]     = lsq_idx[i];
                rob_dispatch_is_store[i]    = dec_is_store[i];
                rob_dispatch_is_load[i]     = dec_is_load[i];
                rob_dispatch_is_branch[i]   = dec_is_branch[i];
                rob_dispatch_is_jal[i]      = dec_is_jal[i];
                rob_dispatch_is_jalr[i]     = dec_is_jalr[i];
                rob_dispatch_has_rd[i]      = dec_has_rd[i];
                rob_dispatch_is_illegal[i]  = dec_illegal[i];
                
                iq_dispatch_en[i]           = 1'b1;
                iq_src1[i]                  = phys_src1[i];
                iq_src2[i]                  = phys_src2[i];
                iq_dst[i]                   = phys_dst[i];
                iq_src1_rdy[i]              = src1_rdy[i];
                iq_src2_rdy[i]              = src2_rdy[i];
                iq_rob_idx[i]               = rob_idx[i];
                iq_alu_op[i]                = dec_alu_op[i];
                iq_imm[i]                   = dec_imm[i];
                iq_pc[i]                    = dec_pc[i];
                iq_funct3[i]                = dec_funct3[i];
                iq_unit_sel[i]              = dec_unit_sel[i];
                iq_uses_imm[i]              = dec_uses_imm[i];
                iq_is_load[i]               = dec_is_load[i];
                iq_is_store[i]              = dec_is_store[i];
                iq_is_branch[i]             = dec_is_branch[i];
                iq_is_jal[i]                = dec_is_jal[i];
                iq_is_jalr[i]               = dec_is_jalr[i];
                iq_is_mul[i]                = dec_is_mul[i];
                iq_is_div[i]                = dec_is_div[i];
                iq_is_word[i]               = dec_is_word[i];
                iq_predicted_taken[i]       = bp_predicted_taken[i];
                iq_predicted_target[i]      = bp_predicted_target[i];
                iq_ghr_snapshot[i]          = bp_current_ghr;
                
                if(dec_is_load[i] || dec_is_store[i])
                begin
                    lsq_dispatch_en[i]          = 1'b1;
                    lsq_dispatch_is_load[i]     = dec_is_load[i];
                    lsq_dispatch_is_store[i]    = dec_is_store[i];
                    lsq_dispatch_rob_idx[i]     = rob_idx[i];
                    lsq_dispatch_phys_dst[i]    = phys_dst[i];
                    lsq_dispatch_mem_size[i]    = dec_mem_size[i];
                    lsq_dispatch_sign_ext[i]    = dec_sign_ext[i];
                end
                else
                    lsq_dispatch_en[i]          = 1'b0;
            end
            else
            begin
                lsq_dispatch_en[i] = 1'b0;
                iq_dispatch_en[i] = 1'b0;
                rob_dispatch_en[i] = 1'b0;
                rat_wr_en[i] = 1'b0;
            end
        end
        
    end
endmodule