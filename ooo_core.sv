`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 18.07.2026 19:54:31
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: ooo_core
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Top-level integration module for the 2-wide superscalar out-of-order
//              RV64G processor core. Instantiates and connects all pipeline stages:
//              fetch, branch prediction, decode, rename/dispatch, reorder buffer,
//              issue queue, physical register file, ALU x2, branch unit, MUL, DIV,
//              load/store address generation unit, load/store queue, and CSR block.
//              Exposes a simple I-cache and D-cache interface to the memory hierarchy.
//
// Dependencies: fetch_unit, branch_predictor, decode_unit, dispatch_unit,
//               free_list, rat, retire_rat, rob, issue_queue, prf,
//               alu_64 (x2), branch_unit, mul_unit, div_unit,
//               ld_st_unit, lsq, csr
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Parameters: DATA_W=64, ISSUE_WIDTH=2, N_PHYS_REGS=96, ROB_ENTRIES=64,
//               IQ_ENTRIES=16, LDQ_ENTRIES=16, STQ_ENTRIES=16, GHR_W=12
//
//////////////////////////////////////////////////////////////////////////////////

module ooo_core #(
    parameter DATA_W       = 64,
    parameter ISSUE_WIDTH  = 2,
    parameter N_PHYS_REGS  = 96,
    parameter N_ARCH_REGS  = 32,
    parameter PHYS_TAG_W   = 7,
    parameter ROB_ENTRIES  = 64,
    parameter ROB_IDX_W    = 6,
    parameter IQ_ENTRIES   = 16,
    parameter LDQ_ENTRIES  = 16,
    parameter STQ_ENTRIES  = 16,
    parameter GHR_W        = 12,
    parameter N_EXEC_UNITS = 4
)(
    input  logic              clk,
    input  logic              rst_n,

    // I-cache interface
    output logic [DATA_W-1:0] icache_pc,
    input  logic [31:0]       icache_instr [0:ISSUE_WIDTH-1],
    input  logic              icache_valid,

    // D-cache interface
    output logic              dcache_rd_req,
    output logic [DATA_W-1:0] dcache_rd_addr,
    input  logic              dcache_rd_valid,
    input  logic [DATA_W-1:0] dcache_rd_data,
    output logic              dcache_wr_req,
    output logic [DATA_W-1:0] dcache_wr_addr,
    output logic [DATA_W-1:0] dcache_wr_data,
    output logic [1:0]        dcache_wr_size
);

    // ─── FLUSH AND STALL ───────────────────────────────────────────
    logic                      flush;
    logic [DATA_W-1:0]         flush_target;
    logic [ROB_IDX_W-1:0]      flush_rob_idx;
    logic                      dispatch_stall;

    // ─── FETCH OUTPUTS ─────────────────────────────────────────────
    logic [DATA_W-1:0]         fetch_current_pc;
    logic [31:0]               fetch_instr        [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    fetch_valid;
    logic [DATA_W-1:0]         fetch_pc           [0:ISSUE_WIDTH-1];

    // ─── BRANCH PREDICTOR OUTPUTS ──────────────────────────────────
    logic                      bp_redirect;
    logic [DATA_W-1:0]         bp_target;
    logic [ISSUE_WIDTH-1:0]    bp_predicted_taken;
    logic [DATA_W-1:0]         bp_predicted_target [0:ISSUE_WIDTH-1];
    logic [GHR_W-1:0]          bp_current_ghr;
    logic                      bp_ckpt_en;
    logic [ROB_IDX_W-1:0]      bp_ckpt_rob_idx;

    // ─── BRANCH PREDICTOR UPDATE ───────────────────────────────────
    logic                      bp_update_en;
    logic [DATA_W-1:0]         bp_update_pc;
    logic                      bp_update_taken;
    logic [DATA_W-1:0]         bp_update_target;
    logic [GHR_W-1:0]          bp_update_ghr;
    logic                      bp_update_is_call;
    logic                      bp_update_is_ret;

    // ─── DECODE OUTPUTS ────────────────────────────────────────────
    logic [ISSUE_WIDTH-1:0]    dec_valid;
    logic [4:0]                dec_rs1        [0:ISSUE_WIDTH-1];
    logic [4:0]                dec_rs2        [0:ISSUE_WIDTH-1];
    logic [4:0]                dec_rd         [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         dec_imm        [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         dec_pc         [0:ISSUE_WIDTH-1];
    logic [3:0]                dec_alu_op     [0:ISSUE_WIDTH-1];
    logic [2:0]                dec_funct3     [0:ISSUE_WIDTH-1];
    logic [1:0]                dec_unit_sel   [0:ISSUE_WIDTH-1];
    logic [1:0]                dec_mem_size   [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    dec_uses_imm;
    logic [ISSUE_WIDTH-1:0]    dec_is_load;
    logic [ISSUE_WIDTH-1:0]    dec_is_store;
    logic [ISSUE_WIDTH-1:0]    dec_is_branch;
    logic [ISSUE_WIDTH-1:0]    dec_is_jalr;
    logic [ISSUE_WIDTH-1:0]    dec_is_jal;
    logic [ISSUE_WIDTH-1:0]    dec_is_mul;
    logic [ISSUE_WIDTH-1:0]    dec_is_div;
    logic [ISSUE_WIDTH-1:0]    dec_sign_ext;
    logic [ISSUE_WIDTH-1:0]    dec_has_rd;
    logic [ISSUE_WIDTH-1:0]    dec_is_word;
    logic [ISSUE_WIDTH-1:0]    dec_illegal;

    // ─── RAT INTERFACE ─────────────────────────────────────────────
    logic [4:0]                rat_rs1_arch   [0:ISSUE_WIDTH-1];
    logic [4:0]                rat_rs2_arch   [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     rat_rs1_phys   [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     rat_rs2_phys   [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    rat_wr_en;
    logic [4:0]                rat_rd_arch    [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     rat_rd_phys    [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     rat_stale_phys [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     retire_rat_map  [0:N_ARCH_REGS-1];
    // Flush restore map: retire_rat_map patched with any simultaneous commit.
    // Needed because retire_rat NBA and rat restore NBA happen in the same delta;
    // the speculative RAT would otherwise restore the pre-commit retire_rat value.
    logic [PHYS_TAG_W-1:0]     flush_rat_map   [0:N_ARCH_REGS-1];

    // ─── FREE LIST INTERFACE ───────────────────────────────────────
    logic [ISSUE_WIDTH-1:0]    fl_pop_req;
    logic [PHYS_TAG_W-1:0]     fl_pop_tag     [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    fl_pop_valid;
    logic                      fl_stall;
    logic                      fl_ckpt_en;
    logic [ROB_IDX_W-1:0]      fl_ckpt_idx;
    logic [1:0]                fl_ckpt_offset;
    logic [ISSUE_WIDTH-1:0]    fl_push_req;
    logic [PHYS_TAG_W-1:0]     fl_push_tag    [0:ISSUE_WIDTH-1];

    // ─── ROB DISPATCH INTERFACE ────────────────────────────────────
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_en;
    logic [DATA_W-1:0]         rob_dispatch_pc       [0:ISSUE_WIDTH-1];
    logic [4:0]                rob_dispatch_arch_rd  [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     rob_dispatch_phys_rd  [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     rob_dispatch_stale    [0:ISSUE_WIDTH-1];
    logic [4:0]                rob_dispatch_lsq_idx  [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_store;
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_load;
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_branch;
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_jal;
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_jalr;
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_has_rd;
    logic [ISSUE_WIDTH-1:0]    rob_dispatch_is_illegal;
    logic [ROB_IDX_W-1:0]      rob_idx            [0:ISSUE_WIDTH-1];
    logic                      rob_full;
    logic                      rob_empty;

    // ─── ROB COMMIT OUTPUTS ────────────────────────────────────────
    logic [ISSUE_WIDTH-1:0]    commit_en;
    logic [4:0]                commit_arch_rd     [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     commit_phys_rd     [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     commit_stale       [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    commit_is_store;
    logic [ISSUE_WIDTH-1:0]    commit_is_load;
    logic [ISSUE_WIDTH-1:0]    commit_has_rd;
    logic [4:0]                commit_lsq_idx     [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         commit_pc          [0:ISSUE_WIDTH-1];
    logic                      commit_exception;
    logic [3:0]                commit_exc_cause;

    // ─── IQ DISPATCH INTERFACE ─────────────────────────────────────
    logic [ISSUE_WIDTH-1:0]    iq_dispatch_en;
    logic [PHYS_TAG_W-1:0]     iq_src1            [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     iq_src2            [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     iq_dst             [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    iq_src1_rdy;
    logic [ISSUE_WIDTH-1:0]    iq_src2_rdy;
    logic [ROB_IDX_W-1:0]      iq_rob_idx         [0:ISSUE_WIDTH-1];
    logic [3:0]                iq_alu_op          [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         iq_imm             [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         iq_pc              [0:ISSUE_WIDTH-1];
    logic [2:0]                iq_funct3          [0:ISSUE_WIDTH-1];
    logic [1:0]                iq_unit_sel        [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    iq_uses_imm;
    logic [ISSUE_WIDTH-1:0]    iq_is_load;
    logic [ISSUE_WIDTH-1:0]    iq_is_store;
    logic [ISSUE_WIDTH-1:0]    iq_is_branch;
    logic [ISSUE_WIDTH-1:0]    iq_is_jal;
    logic [ISSUE_WIDTH-1:0]    iq_is_jalr;
    logic [ISSUE_WIDTH-1:0]    iq_is_mul;
    logic [ISSUE_WIDTH-1:0]    iq_is_div;
    logic [ISSUE_WIDTH-1:0]    iq_is_word;
    logic [1:0]                iq_mem_size        [0:ISSUE_WIDTH-1];
    logic [4:0]                iq_lsq_idx         [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    iq_predicted_taken;
    logic [DATA_W-1:0]         iq_predicted_target [0:ISSUE_WIDTH-1];
    logic [GHR_W-1:0]          iq_ghr_snapshot    [0:ISSUE_WIDTH-1];
    logic                      iq_full;
    logic                      iq_empty;

    // ─── IQ ISSUE OUTPUTS ──────────────────────────────────────────
    logic [N_EXEC_UNITS-1:0]   issue_valid;
    logic [PHYS_TAG_W-1:0]     issue_src1         [0:N_EXEC_UNITS-1];
    logic [PHYS_TAG_W-1:0]     issue_src2         [0:N_EXEC_UNITS-1];
    logic [PHYS_TAG_W-1:0]     issue_dst          [0:N_EXEC_UNITS-1];
    logic [ROB_IDX_W-1:0]      issue_rob_idx      [0:N_EXEC_UNITS-1];
    logic [3:0]                issue_op           [0:N_EXEC_UNITS-1];
    logic [DATA_W-1:0]         issue_imm          [0:N_EXEC_UNITS-1];
    logic [DATA_W-1:0]         issue_pc           [0:N_EXEC_UNITS-1];
    logic [2:0]                issue_funct3       [0:N_EXEC_UNITS-1];
    logic [N_EXEC_UNITS-1:0]   issue_uses_imm;
    logic [N_EXEC_UNITS-1:0]   issue_is_load;
    logic [N_EXEC_UNITS-1:0]   issue_is_store;
    logic [N_EXEC_UNITS-1:0]   issue_is_branch;
    logic [N_EXEC_UNITS-1:0]   issue_is_jal;
    logic [N_EXEC_UNITS-1:0]   issue_is_jalr;
    logic [N_EXEC_UNITS-1:0]   issue_is_mul;
    logic [N_EXEC_UNITS-1:0]   issue_is_div;
    logic [N_EXEC_UNITS-1:0]   issue_is_word;
    logic [1:0]                issue_mem_size     [0:N_EXEC_UNITS-1];
    logic [4:0]                issue_lsq_idx      [0:N_EXEC_UNITS-1];
    logic [N_EXEC_UNITS-1:0]   issue_predicted_taken;
    logic [DATA_W-1:0]         issue_predicted_target [0:N_EXEC_UNITS-1];
    logic [GHR_W-1:0]          issue_ghr_snapshot [0:N_EXEC_UNITS-1];

    // ─── WAKEUP BUS ────────────────────────────────────────────────
    logic [N_EXEC_UNITS-1:0]   wakeup_valid;
    logic [PHYS_TAG_W-1:0]     wakeup_tag         [0:N_EXEC_UNITS-1];
    logic [N_EXEC_UNITS-1:0]   unit_available;

    // ─── PRF INTERFACES ────────────────────────────────────────────
    logic [PHYS_TAG_W-1:0]     prf_rd_tag         [0:N_EXEC_UNITS*2-1];
    logic [DATA_W-1:0]         prf_rd_data        [0:N_EXEC_UNITS*2-1];
    logic                      prf_rd_valid       [0:N_EXEC_UNITS*2-1];
    logic [N_EXEC_UNITS-1:0]   prf_wr_en;
    logic [PHYS_TAG_W-1:0]     prf_wr_tag         [0:N_EXEC_UNITS-1];
    logic [DATA_W-1:0]         prf_wr_data        [0:N_EXEC_UNITS-1];
    logic [ISSUE_WIDTH-1:0]    prf_inv_en;
    logic [PHYS_TAG_W-1:0]     prf_inv_tag        [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     prf_disp_tag       [0:ISSUE_WIDTH*2-1];
    logic                      prf_disp_valid     [0:ISSUE_WIDTH*2-1];

    // ─── ALU OUTPUTS ───────────────────────────────────────────────
    logic [DATA_W-1:0]         alu0_result;
    logic [DATA_W-1:0]         alu1_result;

    // ─── BRANCH UNIT OUTPUTS ───────────────────────────────────────
    logic                      branch_complete;
    logic                      branch_mispredicted;
    logic                      branch_exception;
    logic [3:0]                branch_exc_cause;
    logic [DATA_W-1:0]         branch_actual_target;
    logic [ROB_IDX_W-1:0]      branch_rob_idx_out;
    logic                      branch_rd_wr_en;
    logic [PHYS_TAG_W-1:0]     branch_rd_phys;
    logic [DATA_W-1:0]         branch_rd_value;
    logic                      branch_actual_taken;

    // ─── MUL UNIT OUTPUTS ──────────────────────────────────────────
    logic                      mul_valid_out;
    logic                      mul_complete_out;
    logic [DATA_W-1:0]         mul_result_out;
    logic [ROB_IDX_W-1:0]      mul_rob_idx_out;
    logic [PHYS_TAG_W-1:0]     mul_phys_dst_out;

    // ─── DIV UNIT OUTPUTS ──────────────────────────────────────────
    logic                      div_valid_out;
    logic                      div_complete_out;
    logic [DATA_W-1:0]         div_result_out;
    logic [ROB_IDX_W-1:0]      div_rob_idx_out;
    logic [PHYS_TAG_W-1:0]     div_phys_dst_out;
    logic                      div_unit_available;

    // ─── LD/ST UNIT OUTPUTS ────────────────────────────────────────
    logic                      ldst_complete;
    logic                      ldst_st_complete; // store-only completion — loads complete via lsq_ld_result_valid
    logic [ROB_IDX_W-1:0]      ldst_rob_idx_out;
    logic                      ldst_exception;
    logic [3:0]                ldst_exc_cause;
    logic                      ldst_ld_exec_valid;
    logic [4:0]                ldst_ld_exec_idx;
    logic [DATA_W-1:0]         ldst_ld_exec_addr;
    logic                      ldst_st_exec_valid;
    logic [4:0]                ldst_st_exec_idx;
    logic [DATA_W-1:0]         ldst_st_exec_addr;
    logic [DATA_W-1:0]         ldst_st_exec_data;

    // ─── ALU1 PENDING COMPLETION (port-1 collision buffer) ─────────
    logic                      alu1_pend_v;
    logic [ROB_IDX_W-1:0]     alu1_pend_idx;

    // ─── MUL/DIV/LDST PENDING COMPLETION (port-1 collision buffers) ─
    logic                      mul_pend_v;
    logic [ROB_IDX_W-1:0]     mul_pend_idx;
    logic                      div_pend_v;
    logic [ROB_IDX_W-1:0]     div_pend_idx;
    logic                      ldst_pend_v;
    logic [ROB_IDX_W-1:0]     ldst_pend_idx;
    logic                      ldst_pend_exc;
    logic [3:0]                ldst_pend_cause;
    logic                      ldst_pend2_v;
    logic [ROB_IDX_W-1:0]     ldst_pend2_idx;
    logic                      ldst_pend2_exc;
    logic [3:0]                ldst_pend2_cause;

    // ─── LSQ OUTPUTS ───────────────────────────────────────────────
    logic                      lsq_ld_result_valid;
    logic [PHYS_TAG_W-1:0]     lsq_ld_result_tag;
    logic [DATA_W-1:0]         lsq_ld_result_data;
    logic [ROB_IDX_W-1:0]      lsq_ld_result_rob_idx;
    logic                      lsq_ld_replay;
    logic                      lsq_mem_order_violation;
    logic [ROB_IDX_W-1:0]      lsq_violation_rob_idx;
    logic                      ldq_full;
    logic                      stq_full;

    // ─── LSQ DISPATCH INTERFACE ────────────────────────────────────
    logic [ISSUE_WIDTH-1:0]    lsq_dispatch_en;
    logic [ISSUE_WIDTH-1:0]    lsq_dispatch_is_load;
    logic [ISSUE_WIDTH-1:0]    lsq_dispatch_is_store;
    logic [ROB_IDX_W-1:0]      lsq_dispatch_rob_idx  [0:ISSUE_WIDTH-1];
    logic [PHYS_TAG_W-1:0]     lsq_dispatch_phys_dst [0:ISSUE_WIDTH-1];
    logic [1:0]                lsq_dispatch_mem_size [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    lsq_dispatch_sign_ext;
    logic [4:0]                lsq_ldq_idx           [0:ISSUE_WIDTH-1];
    logic [4:0]                lsq_stq_idx           [0:ISSUE_WIDTH-1];

    // ─── CSR OUTPUTS ───────────────────────────────────────────────
    logic [DATA_W-1:0]         csr_mtvec_out;
    logic [DATA_W-1:0]         csr_mret_pc_out;
    logic [DATA_W-1:0]         csr_mepc_out;
    logic [DATA_W-1:0]         csr_mcause_out;
    logic                      csr_mie;
    logic                      csr_mpie;

    // ─── ROB EXEC COMPLETE BUS ─────────────────────────────────────
    logic [ISSUE_WIDTH-1:0] rob_exec_complete;
    logic [ROB_IDX_W-1:0]   rob_exec_rob_idx [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0] rob_exec_exception;
    logic [3:0]             rob_exec_exc_cause [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0] rob_exec_mispredicted;
    logic [DATA_W-1:0]      rob_exec_branch_target [0:ISSUE_WIDTH-1];

    // ─── MRET SIMPLIFIED ───────────────────────────────────────────
    logic                      mret_en;

    // ===================================================================
    // COMBINATIONAL GLUE
    // ===================================================================

    // Free List push from commit stale tags
    assign fl_push_req = commit_en & commit_has_rd;
    generate
        for(genvar i = 0; i < ISSUE_WIDTH; i++)
            assign fl_push_tag[i] = commit_stale[i];
    endgenerate

    // PRF invalidate from dispatch - new physical destination tag
    assign prf_inv_en = iq_dispatch_en & dec_has_rd;
    generate
        for(genvar i = 0; i < ISSUE_WIDTH; i++)
            assign prf_inv_tag[i] = iq_dst[i];
    endgenerate

    // PRF read ports - 2 sources per execution unit
    generate
        for(genvar u = 0; u < N_EXEC_UNITS; u++)
        begin
            assign prf_rd_tag[u*2]   = issue_src1[u];
            assign prf_rd_tag[u*2+1] = issue_src2[u];
        end
    endgenerate

    // Wakeup bus
    // ALU0 wakes same cycle; JAL/JALR return-address result wakes via branch_rd_wr_en
    assign wakeup_valid[0] = (issue_valid[0] & !issue_is_branch[0] & !issue_is_jal[0] & !issue_is_jalr[0]) | branch_rd_wr_en;
    assign wakeup_tag[0]   = branch_rd_wr_en ? branch_rd_phys : issue_dst[0];
    // ALU1 wakes same cycle
    assign wakeup_valid[1] = issue_valid[1];
    assign wakeup_tag[1]   = issue_dst[1];
    // MULDIV wakes when result is ready (2 cycles for MUL, 32-64 for DIV)
    assign wakeup_valid[2] = mul_valid_out | div_valid_out;
    assign wakeup_tag[2]   = mul_valid_out ? mul_phys_dst_out : div_phys_dst_out;
    // LDST wakes when LSQ delivers load result
    assign wakeup_valid[3] = lsq_ld_result_valid;
    assign wakeup_tag[3]   = lsq_ld_result_tag;

    // Branch return address wakeup - fires same cycle as branch_rd_wr_en
    // This is handled through wakeup port 0 since branch uses ALU0 slot
    // When branch_rd_wr_en fires wakeup[0] carries the branch dst tag

    // Unit availability
    assign unit_available[0] = 1'b1;               // ALU0 always available
    assign unit_available[1] = !alu1_pend_v;       // ALU1 stalls when completion pending
    assign unit_available[2] = div_unit_available; // MULDIV blocks when DIV running
    assign unit_available[3] = 1'b1;               // LDST always available
    // Loads complete via lsq_ld_result_valid; only stores fire ldst_complete to the ROB
    assign ldst_st_complete  = ldst_complete && issue_is_store[3];

    // PRF write bus
    // ------------------------------------------------------------------------
    // FIX: PRF Write Port Multiplexing (Resolves the "Everything is 0" Bug)
    // ------------------------------------------------------------------------
    always_comb
    begin
        // 1. Default all write ports to 0
        for(int i = 0; i < 3'd4; i++)
        begin
            prf_wr_en[i]   = 1'b0;
            prf_wr_tag[i]  = '0;
            prf_wr_data[i] = '0;
        end

        // 2. Port 0: ALU0 & Branch Unit (Issue Slot 0)
        // The Branch unit takes precedence for JAL/JALR to write the return address
        if (issue_valid[0] && (issue_is_branch[0] || issue_is_jal[0] || issue_is_jalr[0]))
        begin
            prf_wr_en[0]   = branch_rd_wr_en;
            prf_wr_tag[0]  = branch_rd_phys;
            prf_wr_data[0] = branch_rd_value;
        end
        else if (issue_valid[0]) // Standard ALU operation (any op code)
        begin
            prf_wr_en[0]   = 1'b1;
            prf_wr_tag[0]  = issue_dst[0];
            prf_wr_data[0] = alu0_result;
        end

        // 3. Port 1: ALU1 (Issue Slot 1, unit_sel==2'b00)
        if (issue_valid[1])
        begin
            prf_wr_en[1]   = 1'b1;
            prf_wr_tag[1]  = issue_dst[1];
            prf_wr_data[1] = alu1_result;
        end

        // 4. Port 2: MUL / DIV Unit
        if (mul_valid_out)
        begin
            prf_wr_en[2]   = 1'b1;
            prf_wr_tag[2]  = mul_phys_dst_out;
            prf_wr_data[2] = mul_result_out;
        end
        else if (div_valid_out)
        begin
            prf_wr_en[2]   = 1'b1;
            prf_wr_tag[2]  = div_phys_dst_out;
            prf_wr_data[2] = div_result_out;
        end

        // 5. Port 3: Load/Store Queue (Loads returning from D-Cache or forwarding)
        if (lsq_ld_result_valid)
        begin
            prf_wr_en[3]   = 1'b1;
            prf_wr_tag[3]  = lsq_ld_result_tag;
            prf_wr_data[3] = lsq_ld_result_data;
        end
    end

    // ROB exec complete bus is driven entirely by the always_comb arbiter below.

    // Port 1: ALU1, MULDIV, LDST, LSQ load result
    // When ALU1 and another unit both complete simultaneously, ALU1's rob_idx would be
    // dropped by the priority mux.  We save it and replay it next cycle at highest priority.
    always_ff @(posedge clk) begin
        if (!rst_n || flush) begin
            alu1_pend_v   <= 1'b0;
            alu1_pend_idx <= '0;
        end else begin
            // consumed = alu1_pend is being served this cycle (nothing higher-priority on port 1)
            automatic logic alu1_pend_consumed =
                alu1_pend_v && !lsq_ld_result_valid && !mul_pend_v &&
                !ldst_pend_v && !ldst_pend2_v && !div_pend_v &&
                !ldst_st_complete && !mul_complete_out && !div_complete_out;
            if (issue_valid[1] &&
                (lsq_ld_result_valid | ldst_st_complete | mul_complete_out | div_complete_out |
                 ldst_pend_v | ldst_pend2_v | mul_pend_v | div_pend_v)) begin
                // ALU1 wants port 1 but it is occupied — save (unit_available gating ensures no overwrite)
                alu1_pend_v   <= 1'b1;
                alu1_pend_idx <= issue_rob_idx[1];
            end else if (alu1_pend_consumed) begin
                alu1_pend_v   <= 1'b0;  // served — release slot
            end
            // else: hold until served
        end
    end

    // Save MUL/DIV/LDST completions dropped from port 1 by higher-priority signals.
    // Port-1 priority (new): lsq > mul_pend > ldst_pend > ldst_pend2 > div_pend >
    //                        ldst_complete > mul_complete > div_complete > alu1_pend > alu1
    //
    // mul_pend_consumed: mul_pend is being served by port-1 THIS cycle (nothing higher preempts it)
    // ldst_pend_consumed: ldst_pend is being served this cycle (lsq and mul_pend both absent)
    // ldst_pend2_consumed: ldst_pend2 is being served this cycle (lsq, mul_pend, ldst_pend all absent)
    always_ff @(posedge clk) begin
        if (!rst_n || flush) begin
            mul_pend_v     <= 1'b0;  mul_pend_idx   <= '0;
            div_pend_v     <= 1'b0;  div_pend_idx   <= '0;
            ldst_pend_v    <= 1'b0;  ldst_pend_idx  <= '0;
            ldst_pend_exc  <= 1'b0;  ldst_pend_cause <= '0;
            ldst_pend2_v   <= 1'b0;  ldst_pend2_idx <= '0;
            ldst_pend2_exc <= 1'b0;  ldst_pend2_cause <= '0;
        end else begin
            // Consume flags: true when port-1 serves that slot this cycle
            automatic logic mul_pend_consumed  = mul_pend_v  && !lsq_ld_result_valid;
            automatic logic ldst_pend_consumed = ldst_pend_v && !lsq_ld_result_valid && !mul_pend_v;
            automatic logic ldst_pend2_consumed= ldst_pend2_v&& !lsq_ld_result_valid && !mul_pend_v && !ldst_pend_v;

            // MUL: save when mul_complete is preempted AND slot is free (empty or just consumed)
            if (mul_complete_out &&
                (lsq_ld_result_valid | ldst_pend_v | ldst_pend2_v | div_pend_v | ldst_st_complete) &&
                (!mul_pend_v || mul_pend_consumed)) begin
                mul_pend_v   <= 1'b1;
                mul_pend_idx <= mul_rob_idx_out;
            end else if (mul_pend_consumed) begin
                mul_pend_v   <= 1'b0;
            end
            // else: hold (lsq preempts mul_pend's port-1 turn)

            // DIV: save when div_complete is preempted AND slot is free
            if (div_complete_out &&
                (lsq_ld_result_valid | mul_pend_v | ldst_pend_v | ldst_pend2_v | div_pend_v | ldst_st_complete | mul_complete_out) &&
                (!div_pend_v || div_pend_v && !lsq_ld_result_valid && !mul_pend_v && !ldst_pend_v && !ldst_pend2_v)) begin
                div_pend_v   <= 1'b1;
                div_pend_idx <= div_rob_idx_out;
            end else if (div_pend_v && !lsq_ld_result_valid && !mul_pend_v && !ldst_pend_v && !ldst_pend2_v) begin
                div_pend_v   <= 1'b0;
            end
            // else: hold

            // LDST slot 0: save when ldst_st_complete is preempted AND slot 0 is free (empty or consumed)
            if (ldst_st_complete &&
                (lsq_ld_result_valid | mul_pend_v | ldst_pend2_v | div_pend_v | ldst_pend_v) &&
                (!ldst_pend_v || ldst_pend_consumed)) begin
                ldst_pend_v    <= 1'b1;
                ldst_pend_idx  <= ldst_rob_idx_out;
                ldst_pend_exc  <= ldst_exception;
                ldst_pend_cause <= ldst_exc_cause;
            end else if (ldst_pend_consumed) begin
                ldst_pend_v    <= 1'b0;
            end
            // else: hold

            // LDST slot 1: save when ldst_st_complete is preempted, slot 0 is occupied (not consumed),
            //              AND slot 1 is free (empty or consumed)
            if (ldst_st_complete &&
                (lsq_ld_result_valid | mul_pend_v | ldst_pend2_v | div_pend_v | ldst_pend_v) &&
                ldst_pend_v && !ldst_pend_consumed &&
                (!ldst_pend2_v || ldst_pend2_consumed)) begin
                ldst_pend2_v    <= 1'b1;
                ldst_pend2_idx  <= ldst_rob_idx_out;
                ldst_pend2_exc  <= ldst_exception;
                ldst_pend2_cause <= ldst_exc_cause;
            end else if (ldst_pend2_consumed) begin
                ldst_pend2_v    <= 1'b0;
            end
            // else: hold
        end
    end

    // ------------------------------------------------------------------------
    // FIX: 2-Port ROB Completion Arbiter (Resolves the Multiple Drivers)
    // ------------------------------------------------------------------------
    always_comb
    begin
        // 1. Initialize both ports to 0
        for(int i = 0; i < ISSUE_WIDTH; i++) begin
            rob_exec_complete[i]      = 1'b0;
            rob_exec_rob_idx[i]       = '0;
            rob_exec_exception[i]     = 1'b0;
            rob_exec_exc_cause[i]     = '0;
            rob_exec_mispredicted[i]  = 1'b0;
            rob_exec_branch_target[i] = '0;
        end

        // 2. PORT 0: Branch Unit > ALU0
        if (branch_complete) begin
            rob_exec_complete[0]      = 1'b1;
            rob_exec_rob_idx[0]       = branch_rob_idx_out;
            rob_exec_exception[0]     = branch_exception;
            rob_exec_exc_cause[0]     = branch_exc_cause;
            rob_exec_mispredicted[0]  = branch_mispredicted;
            rob_exec_branch_target[0] = branch_actual_target;
        end
        else if (issue_valid[0]) begin
            rob_exec_complete[0]      = 1'b1;
            rob_exec_rob_idx[0]       = issue_rob_idx[0];
        end

        // 3. PORT 1: Cache Return > MUL_pend > LDST_pend > LDST_pend2 > DIV_pend >
        //            LDST > MUL > DIV > pending ALU1 > ALU1
        // MUL_pend has priority over LDST_pend so 3 consecutive MUL completions don't overwrite
        // each other: each is consumed before the next arrives.
        if (lsq_ld_result_valid) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = lsq_ld_result_rob_idx;
        end
        else if (mul_pend_v) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = mul_pend_idx;
        end
        else if (ldst_pend_v) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = ldst_pend_idx;
            rob_exec_exception[1]     = ldst_pend_exc;
            rob_exec_exc_cause[1]     = ldst_pend_cause;
        end
        else if (ldst_pend2_v) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = ldst_pend2_idx;
            rob_exec_exception[1]     = ldst_pend2_exc;
            rob_exec_exc_cause[1]     = ldst_pend2_cause;
        end
        else if (div_pend_v) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = div_pend_idx;
        end
        else if (ldst_st_complete) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = ldst_rob_idx_out;
            rob_exec_exception[1]     = ldst_exception;
            rob_exec_exc_cause[1]     = ldst_exc_cause;
        end
        else if (mul_complete_out) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = mul_rob_idx_out;
        end
        else if (div_complete_out) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = div_rob_idx_out;
        end
        else if (alu1_pend_v) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = alu1_pend_idx;
        end
        else if (issue_valid[1]) begin
            rob_exec_complete[1]      = 1'b1;
            rob_exec_rob_idx[1]       = issue_rob_idx[1];
        end
    end

    // Branch predictor update from branch unit
    assign bp_update_en      = branch_complete;
    assign bp_update_pc      = issue_pc[0];
    assign bp_update_taken   = branch_actual_taken;
    assign bp_update_target  = branch_actual_target;
    assign bp_update_ghr     = issue_ghr_snapshot[0]; // now correctly wired from IQ
    assign bp_update_is_call = issue_is_jal[0];
    assign bp_update_is_ret  = issue_is_jalr[0];

    // IQ dispatch: pass lsq_idx from LSQ back through dispatch for IQ storage
    // lsq_idx is muxed in dispatch_unit into iq_lsq_idx
    generate
        for(genvar i = 0; i < ISSUE_WIDTH; i++)
            assign iq_lsq_idx[i] = dec_is_load[i]  ? lsq_ldq_idx[i] :
                                   dec_is_store[i] ? lsq_stq_idx[i] : 5'd0;
    endgenerate

    // IQ dispatch mem_size from decode
    generate
        for(genvar i = 0; i < ISSUE_WIDTH; i++)
            assign iq_mem_size[i] = dec_mem_size[i];
    endgenerate

    // MRET simplified
    assign mret_en = 1'b0;

    // ===================================================================
    // MODULE INSTANTIATIONS
    // ===================================================================

    fetch_unit #(
        .DATA_W     (DATA_W),
        .ISSUE_WIDTH(ISSUE_WIDTH)
    ) u_fetch (
        .clk         (clk),
        .rst_n       (rst_n),
        .flush       (flush),
        .flush_target(flush_target),
        .bp_redirect (bp_redirect),
        .bp_target   (bp_target),
        .stall       (dispatch_stall),
        .icache_pc   (icache_pc),
        .icache_instr(icache_instr),
        .icache_valid(icache_valid),
        .fetch_valid (fetch_valid),
        .fetch_instr (fetch_instr),
        .fetch_pc    (fetch_pc),
        .current_pc  (fetch_current_pc)
    );

    branch_predictor #(
        .DATA_W     (DATA_W),
        .GHR_W      (GHR_W),
        .ROB_ENTRIES(ROB_ENTRIES),
        .ROB_IDX_W  (ROB_IDX_W),
        .ISSUE_WIDTH(ISSUE_WIDTH)
    ) u_bp (
        .clk              (clk),
        .rst_n            (rst_n),
        .fetch_pc         (fetch_current_pc),
        .fetch_instr      (fetch_instr),
        .fetch_valid      (fetch_valid),
        .bp_redirect      (bp_redirect),
        .bp_target        (bp_target),
        .predicted_taken  (bp_predicted_taken),
        .predicted_target (bp_predicted_target),
        .current_ghr      (bp_current_ghr),
        .ckpt_en          (bp_ckpt_en),
        .ckpt_rob_idx     (bp_ckpt_rob_idx),
        .update_en        (bp_update_en),
        .update_pc        (bp_update_pc),
        .update_taken     (bp_update_taken),
        .update_target    (bp_update_target),
        .update_ghr       (bp_update_ghr),
        .update_is_call   (bp_update_is_call),
        .update_is_ret    (bp_update_is_ret),
        .flush            (flush),
        .flush_rob_idx    (flush_rob_idx)
    );

    decode_unit #(
        .DATA_W     (DATA_W),
        .ISSUE_WIDTH(ISSUE_WIDTH)
    ) u_decode (
        .clk            (clk),
        .rst_n          (rst_n),
        .fetch_valid    (fetch_valid),
        .fetch_instr    (fetch_instr),
        .fetch_pc       (fetch_pc),
        .stall          (dispatch_stall),
        .flush          (flush),
        .dec_valid      (dec_valid),
        .dec_rs1        (dec_rs1),
        .dec_rs2        (dec_rs2),
        .dec_rd         (dec_rd),
        .dec_imm        (dec_imm),
        .dec_pc         (dec_pc),
        .dec_alu_op     (dec_alu_op),
        .dec_funct3     (dec_funct3),
        .dec_unit_sel   (dec_unit_sel),
        .dec_mem_size   (dec_mem_size),
        .dec_uses_imm   (dec_uses_imm),
        .dec_is_load    (dec_is_load),
        .dec_is_store   (dec_is_store),
        .dec_is_branch  (dec_is_branch),
        .dec_is_jalr    (dec_is_jalr),
        .dec_is_jal     (dec_is_jal),
        .dec_is_mul     (dec_is_mul),
        .dec_is_div     (dec_is_div),
        .dec_sign_ext   (dec_sign_ext),
        .dec_has_rd     (dec_has_rd),
        .dec_is_word    (dec_is_word),
        .dec_illegal    (dec_illegal)
    );

    free_list #(
        .N_PHYS_REGS(N_PHYS_REGS),
        .N_ARCH_REGS(N_ARCH_REGS),
        .ISSUE_WIDTH(ISSUE_WIDTH),
        .PHYS_TAG_W (PHYS_TAG_W),
        .ROB_ENTRIES(ROB_ENTRIES),
        .ROB_IDX_W  (ROB_IDX_W)
    ) u_fl (
        .clk           (clk),
        .rst_n         (rst_n),
        .pop_req       (fl_pop_req),
        .pop_tag       (fl_pop_tag),
        .pop_valid     (fl_pop_valid),
        .push_req      (fl_push_req),
        .push_tag      (fl_push_tag),
        .ckpt_en       (fl_ckpt_en),
        .ckpt_idx      (fl_ckpt_idx),
        .ckpt_offset   (fl_ckpt_offset),
        .flush         (flush),
        .flush_rob_idx (flush_rob_idx),
        .fl_count      (),
        .stall         (fl_stall)
    );

    rat #(
        .N_ARCH_REGS(N_ARCH_REGS),
        .N_PHYS_REGS(N_PHYS_REGS),
        .PHYS_TAG_W (PHYS_TAG_W),
        .ISSUE_WIDTH(ISSUE_WIDTH)
    ) u_rat (
        .clk           (clk),
        .rst_n         (rst_n),
        .rs1_arch      (rat_rs1_arch),
        .rs2_arch      (rat_rs2_arch),
        .rs1_phys      (rat_rs1_phys),
        .rs2_phys      (rat_rs2_phys),
        .wr_en         (rat_wr_en),
        .rd_arch       (rat_rd_arch),
        .rd_phys       (rat_rd_phys),
        .flush         (flush),
        .retire_rat_in (flush_rat_map),
        .stale_phys    (rat_stale_phys)
    );

    // Build the flush-restore map: start from retire_rat, then apply any commit
    // that is happening in the same cycle (JAL/JALR writes rd even on flush).
    always_comb begin
        for (int i = 0; i < N_ARCH_REGS; i++)
            flush_rat_map[i] = retire_rat_map[i];
        for (int i = 0; i < ISSUE_WIDTH; i++) begin
            if (commit_en[i] && commit_has_rd[i] && (commit_arch_rd[i] != '0))
                flush_rat_map[commit_arch_rd[i]] = commit_phys_rd[i];
        end
    end

    retire_rat #(
        .N_ARCH_REGS(N_ARCH_REGS),
        .N_PHYS_REGS(N_PHYS_REGS),
        .PHYS_TAG_W (PHYS_TAG_W),
        .ISSUE_WIDTH(ISSUE_WIDTH)
    ) u_retire_rat (
        .clk            (clk),
        .rst_n          (rst_n),
        .commit_en      (commit_en),
        .commit_arch_rd (commit_arch_rd),
        .commit_phys_rd (commit_phys_rd),
        .retire_map     (retire_rat_map)
    );

    rob #(
        .ROB_ENTRIES(ROB_ENTRIES),
        .N_ARCH_REGS(N_ARCH_REGS),
        .PHYS_TAG_W (PHYS_TAG_W),
        .ISSUE_WIDTH(ISSUE_WIDTH),
        .DATA_W     (DATA_W),
        .LSQ_ENTRIES(LDQ_ENTRIES)
    ) u_rob (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .dispatch_en           (rob_dispatch_en),
        .dispatch_pc           (rob_dispatch_pc),
        .dispatch_arch_rd      (rob_dispatch_arch_rd),
        .dispatch_phys_rd      (rob_dispatch_phys_rd),
        .dispatch_stale        (rob_dispatch_stale),
        .dispatch_lsq_idx      (rob_dispatch_lsq_idx),
        .dispatch_is_store     (rob_dispatch_is_store),
        .dispatch_is_load      (rob_dispatch_is_load),
        .dispatch_is_branch    (rob_dispatch_is_branch),
        .dispatch_has_rd       (rob_dispatch_has_rd),
        .exec_complete         (rob_exec_complete),
        .exec_rob_idx          (rob_exec_rob_idx),
        .exec_exception        (rob_exec_exception),
        .exec_exc_cause        (rob_exec_exc_cause),
        .exec_mispredicted     (rob_exec_mispredicted),
        .exec_branch_target    (rob_exec_branch_target),
        .trap_handler_pc       (csr_mtvec_out),
        .commit_en             (commit_en),
        .commit_arch_rd        (commit_arch_rd),
        .commit_phys_rd        (commit_phys_rd),
        .commit_stale          (commit_stale),
        .commit_is_store       (commit_is_store),
        .commit_is_load        (commit_is_load),
        .commit_has_rd         (commit_has_rd),
        .commit_lsq_idx        (commit_lsq_idx),
        .commit_pc             (commit_pc),
        .commit_exception      (commit_exception),
        .commit_exc_cause      (commit_exc_cause),
        .flush                 (flush),
        .flush_target          (flush_target),
        .flush_rob_idx         (flush_rob_idx),
        .rob_idx               (rob_idx),
        .rob_full              (rob_full),
        .rob_empty             (rob_empty)
    );

    issue_queue #(
        .IQ_ENTRIES  (IQ_ENTRIES),
        .PHYS_TAG_W  (PHYS_TAG_W),
        .ISSUE_WIDTH (ISSUE_WIDTH),
        .ROB_IDX_W   (ROB_IDX_W),
        .DATA_W      (DATA_W),
        .N_EXEC_UNITS(N_EXEC_UNITS),
        .GHR_W       (GHR_W)
    ) u_iq (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .dispatch_en               (iq_dispatch_en),
        .dispatch_src1             (iq_src1),
        .dispatch_src2             (iq_src2),
        .dispatch_dst              (iq_dst),
        .dispatch_src1_rdy         (iq_src1_rdy),
        .dispatch_src2_rdy         (iq_src2_rdy),
        .dispatch_rob_idx          (iq_rob_idx),
        .dispatch_op               (iq_alu_op),
        .dispatch_imm              (iq_imm),
        .dispatch_pc               (iq_pc),
        .dispatch_funct3           (iq_funct3),
        .dispatch_uses_imm         (iq_uses_imm),
        .dispatch_is_load          (iq_is_load),
        .dispatch_is_store         (iq_is_store),
        .dispatch_is_branch        (iq_is_branch),
        .dispatch_is_jal           (iq_is_jal),
        .dispatch_is_jalr          (iq_is_jalr),
        .dispatch_is_mul           (iq_is_mul),
        .dispatch_is_div           (iq_is_div),
        .dispatch_is_word          (iq_is_word),
        .dispatch_mem_size         (iq_mem_size),
        .dispatch_lsq_idx          (iq_lsq_idx),
        .dispatch_predicted_taken  (iq_predicted_taken),
        .dispatch_predicted_target (iq_predicted_target),
        .dispatch_ghr_snapshot     (iq_ghr_snapshot),
        .dispatch_unit_sel         (iq_unit_sel),
        .wakeup_valid              (wakeup_valid),
        .wakeup_tag                (wakeup_tag),
        .unit_available            (unit_available),
        .issue_valid               (issue_valid),
        .issue_src1                (issue_src1),
        .issue_src2                (issue_src2),
        .issue_dst                 (issue_dst),
        .issue_rob_idx             (issue_rob_idx),
        .issue_op                  (issue_op),
        .issue_imm                 (issue_imm),
        .issue_pc                  (issue_pc),
        .issue_funct3              (issue_funct3),
        .issue_uses_imm            (issue_uses_imm),
        .issue_is_load             (issue_is_load),
        .issue_is_store            (issue_is_store),
        .issue_is_branch           (issue_is_branch),
        .issue_is_jal              (issue_is_jal),
        .issue_is_jalr             (issue_is_jalr),
        .issue_is_mul              (issue_is_mul),
        .issue_is_div              (issue_is_div),
        .issue_is_word             (issue_is_word),
        .issue_mem_size            (issue_mem_size),
        .issue_lsq_idx             (issue_lsq_idx),
        .issue_predicted_taken     (issue_predicted_taken),
        .issue_predicted_target    (issue_predicted_target),
        .issue_ghr_snapshot        (issue_ghr_snapshot),
        .flush                     (flush),
        .ld_replay                 (lsq_ld_replay),
        .iq_full                   (iq_full),
        .iq_empty                  (iq_empty)
    );

    prf #(
        .N_PHYS_REGS  (N_PHYS_REGS),
        .DATA_W       (DATA_W),
        .PHYS_TAG_W   (PHYS_TAG_W),
        .N_EXEC_UNITS (N_EXEC_UNITS),
        .N_READ_PORTS (N_EXEC_UNITS*2),
        .N_WRITE_PORTS(N_EXEC_UNITS),
        .ISSUE_WIDTH  (ISSUE_WIDTH)
    ) u_prf (
        .clk       (clk),
        .rst_n     (rst_n),
        .rd_tag    (prf_rd_tag),
        .rd_data   (prf_rd_data),
        .rd_valid  (prf_rd_valid),
        .wr_en     (prf_wr_en),
        .wr_tag    (prf_wr_tag),
        .wr_data   (prf_wr_data),
        .inv_en    (prf_inv_en),
        .inv_tag   (prf_inv_tag),
        .disp_tag  (prf_disp_tag),
        .disp_valid(prf_disp_valid),
        .flush     (flush)
    );

    dispatch_unit #(
        .DATA_W     (DATA_W),
        .ISSUE_WIDTH(ISSUE_WIDTH),
        .PHYS_TAG_W (PHYS_TAG_W),
        .ROB_IDX_W  (ROB_IDX_W),
        .GHR_W      (GHR_W)
    ) u_dispatch (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .flush                   (flush),
        .dec_valid               (dec_valid),
        .dec_rs1                 (dec_rs1),
        .dec_rs2                 (dec_rs2),
        .dec_rd                  (dec_rd),
        .dec_imm                 (dec_imm),
        .dec_pc                  (dec_pc),
        .dec_alu_op              (dec_alu_op),
        .dec_funct3              (dec_funct3),
        .dec_unit_sel            (dec_unit_sel),
        .dec_mem_size            (dec_mem_size),
        .dec_uses_imm            (dec_uses_imm),
        .dec_is_load             (dec_is_load),
        .dec_is_store            (dec_is_store),
        .dec_is_branch           (dec_is_branch),
        .dec_is_jalr             (dec_is_jalr),
        .dec_is_jal              (dec_is_jal),
        .dec_is_mul              (dec_is_mul),
        .dec_is_div              (dec_is_div),
        .dec_sign_ext            (dec_sign_ext),
        .dec_has_rd              (dec_has_rd),
        .dec_is_word             (dec_is_word),
        .dec_illegal             (dec_illegal),
        .rat_rs1_arch            (rat_rs1_arch),
        .rat_rs2_arch            (rat_rs2_arch),
        .rat_rs1_phys            (rat_rs1_phys),
        .rat_rs2_phys            (rat_rs2_phys),
        .rat_wr_en               (rat_wr_en),
        .rat_rd_arch             (rat_rd_arch),
        .rat_rd_phys             (rat_rd_phys),
        .rat_stale_phys          (rat_stale_phys),
        .fl_pop_req              (fl_pop_req),
        .fl_pop_tag              (fl_pop_tag),
        .fl_pop_valid            (fl_pop_valid),
        .fl_stall                (fl_stall),
        .fl_ckpt_en              (fl_ckpt_en),
        .fl_ckpt_idx             (fl_ckpt_idx),
        .fl_ckpt_offset          (fl_ckpt_offset),
        .rob_dispatch_en         (rob_dispatch_en),
        .rob_dispatch_pc         (rob_dispatch_pc),
        .rob_dispatch_arch_rd    (rob_dispatch_arch_rd),
        .rob_dispatch_phys_rd    (rob_dispatch_phys_rd),
        .rob_dispatch_stale      (rob_dispatch_stale),
        .rob_dispatch_lsq_idx    (rob_dispatch_lsq_idx),
        .rob_dispatch_is_store   (rob_dispatch_is_store),
        .rob_dispatch_is_load    (rob_dispatch_is_load),
        .rob_dispatch_is_branch  (rob_dispatch_is_branch),
        .rob_dispatch_is_jal     (rob_dispatch_is_jal),
        .rob_dispatch_is_jalr    (rob_dispatch_is_jalr),
        .rob_dispatch_has_rd     (rob_dispatch_has_rd),
        .rob_dispatch_is_illegal (rob_dispatch_is_illegal),
        .rob_idx                 (rob_idx),
        .rob_full                (rob_full),
        .iq_dispatch_en          (iq_dispatch_en),
        .iq_src1                 (iq_src1),
        .iq_src2                 (iq_src2),
        .iq_dst                  (iq_dst),
        .iq_src1_rdy             (iq_src1_rdy),
        .iq_src2_rdy             (iq_src2_rdy),
        .iq_rob_idx              (iq_rob_idx),
        .iq_alu_op               (iq_alu_op),
        .iq_imm                  (iq_imm),
        .iq_pc                   (iq_pc),
        .iq_funct3               (iq_funct3),
        .iq_unit_sel             (iq_unit_sel),
        .iq_uses_imm             (iq_uses_imm),
        .iq_is_load              (iq_is_load),
        .iq_is_store             (iq_is_store),
        .iq_is_branch            (iq_is_branch),
        .iq_is_jal               (iq_is_jal),
        .iq_is_jalr              (iq_is_jalr),
        .iq_is_mul               (iq_is_mul),
        .iq_is_div               (iq_is_div),
        .iq_is_word              (iq_is_word),
        .iq_predicted_taken      (iq_predicted_taken),
        .iq_predicted_target     (iq_predicted_target),
        .iq_ghr_snapshot         (iq_ghr_snapshot),
        .iq_full                 (iq_full),
        .lsq_dispatch_en         (lsq_dispatch_en),
        .lsq_dispatch_is_load    (lsq_dispatch_is_load),
        .lsq_dispatch_is_store   (lsq_dispatch_is_store),
        .lsq_dispatch_rob_idx    (lsq_dispatch_rob_idx),
        .lsq_dispatch_phys_dst   (lsq_dispatch_phys_dst),
        .lsq_dispatch_mem_size   (lsq_dispatch_mem_size),
        .lsq_dispatch_sign_ext   (lsq_dispatch_sign_ext),
        .lsq_ldq_idx             (lsq_ldq_idx),
        .lsq_stq_idx             (lsq_stq_idx),
        .ldq_full                (ldq_full),
        .stq_full                (stq_full),
        .prf_disp_tag            (prf_disp_tag),
        .prf_disp_valid          (prf_disp_valid),
        .bp_current_ghr          (bp_current_ghr),
        .bp_predicted_taken      (bp_predicted_taken),
        .bp_predicted_target     (bp_predicted_target),
        .bp_ckpt_en              (bp_ckpt_en),
        .bp_ckpt_rob_idx         (bp_ckpt_rob_idx),
        .dispatch_stall          (dispatch_stall)
    );
    
    // ------------------------------------------------------------------------
    // FIX: ALU Immediate Routing
    // ------------------------------------------------------------------------
    logic [DATA_W-1:0] alu0_src2_actual;
    logic [DATA_W-1:0] alu1_src2_actual;

    always_comb
    begin
        // Route immediate if uses_imm is true, otherwise route PRF read data
        alu0_src2_actual = issue_uses_imm[0] ? issue_imm[0] : prf_rd_data[1]; // Assuming port 1 is src2 for slot 0
        alu1_src2_actual = issue_uses_imm[1] ? issue_imm[1] : prf_rd_data[3]; // Assuming port 3 is src2 for slot 1
    end
    
    // ALU0 - unit index 0
    alu_64 #(.DATA_W(DATA_W)) u_alu0 (
        .src1   (prf_rd_data[0]),
        .src2   (alu0_src2_actual),
        .alu_op (issue_op[0]),
        .is_word(issue_is_word[0]),  // now correctly wired from IQ
        .result (alu0_result)
    );

    // ALU1 - unit index 1
    alu_64 #(.DATA_W(DATA_W)) u_alu1 (
        .src1   (prf_rd_data[2]),
        .src2   (alu1_src2_actual),
        .alu_op (issue_op[1]),
        .is_word(issue_is_word[1]),  // now correctly wired from IQ
        .result (alu1_result)
    );

    // Branch unit - shares ALU0 issue slot (unit index 0)
    branch_unit #(
        .DATA_W    (DATA_W),
        .ROB_IDX_W (ROB_IDX_W),
        .PHYS_TAG_W(PHYS_TAG_W)
    ) u_branch (
        .valid            (issue_valid[0] & (issue_is_branch[0] | issue_is_jal[0] | issue_is_jalr[0])),
        .src1             (prf_rd_data[0]),
        .src2             (prf_rd_data[1]),
        .imm              (issue_imm[0]),
        .instr_pc         (issue_pc[0]),
        .funct3           (issue_funct3[0]),   // now correctly wired from IQ
        .is_jalr          (issue_is_jalr[0]),  // now correctly wired from IQ
        .is_jal           (issue_is_jal[0]),   // now correctly wired from IQ
        .predicted_taken  (issue_predicted_taken[0]),
        .predicted_target (issue_predicted_target[0]),
        .rob_idx          (issue_rob_idx[0]),
        .phys_dst         (issue_dst[0]),
        .complete         (branch_complete),
        .mispredicted     (branch_mispredicted),
        .exception        (branch_exception),
        .exc_cause        (branch_exc_cause),
        .actual_target    (branch_actual_target),
        .result_rob_idx   (branch_rob_idx_out),
        .rd_wr_en         (branch_rd_wr_en),
        .rd_phys          (branch_rd_phys),
        .rd_value         (branch_rd_value),
        .actual_taken     (branch_actual_taken)
    );

    // MUL unit - unit index 2 - MUL ops have is_mul=1
    mul_unit #(
        .DATA_W    (DATA_W),
        .ROB_IDX_W (ROB_IDX_W),
        .PHYS_TAG_W(PHYS_TAG_W)
    ) u_mul (
        .clk              (clk),
        .rst_n            (rst_n),
        .flush            (flush),
        .valid_in         (issue_valid[2] & issue_is_mul[2]),  // now correctly gated by is_mul
        .src1_in          (prf_rd_data[4]),
        .src2_in          (prf_rd_data[5]),
        .mul_op_in        (issue_op[2][1:0]),
        .is_word_in       (issue_is_word[2]),                  // now correctly wired from IQ
        .rob_idx_in       (issue_rob_idx[2]),
        .phys_dst_in      (issue_dst[2]),
        .valid_out        (mul_valid_out),
        .complete_out     (mul_complete_out),
        .mispredicted_out (),
        .exception_out    (),
        .exc_cause_out    (),
        .branch_target_out(),
        .result_out       (mul_result_out),
        .rob_idx_out      (mul_rob_idx_out),
        .phys_dst_out     (mul_phys_dst_out),
        .unit_available   ()
    );

    // DIV unit - unit index 2 - DIV ops have is_div=1
    div_unit #(
        .DATA_W    (DATA_W),
        .ROB_IDX_W (ROB_IDX_W),
        .PHYS_TAG_W(PHYS_TAG_W)
    ) u_div (
        .clk              (clk),
        .rst_n            (rst_n),
        .flush            (flush),
        .valid_in         (issue_valid[2] & issue_is_div[2]),  // now correctly gated by is_div
        .src1_in          (prf_rd_data[4]),
        .src2_in          (prf_rd_data[5]),
        .div_op_in        (issue_op[2][1:0]),
        .is_word_in       (issue_is_word[2]),                  // now correctly wired from IQ
        .rob_idx_in       (issue_rob_idx[2]),
        .phys_dst_in      (issue_dst[2]),
        .valid_out        (div_valid_out),
        .complete_out     (div_complete_out),
        .mispredicted_out (),
        .exception_out    (),
        .exc_cause_out    (),
        .branch_target_out(),
        .result_out       (div_result_out),
        .rob_idx_out      (div_rob_idx_out),
        .phys_dst_out     (div_phys_dst_out),
        .unit_available   (div_unit_available)
    );

    // LD/ST unit - unit index 3
    ld_st_unit #(
        .DATA_W    (DATA_W),
        .ROB_IDX_W (ROB_IDX_W),
        .PHYS_TAG_W(PHYS_TAG_W)
    ) u_ldst (
        .valid_in         (issue_valid[3]),
        .src1_in          (prf_rd_data[6]),
        .src2_in          (prf_rd_data[7]),
        .imm_in           (issue_imm[3]),
        .is_load_in       (issue_is_load[3]),
        .is_store_in      (issue_is_store[3]),
        .mem_size_in      (issue_mem_size[3]),  // now correctly wired from IQ
        .rob_idx_in       (issue_rob_idx[3]),
        .lsq_idx_in       (issue_lsq_idx[3]),  // now correctly wired from IQ
        .ld_exec_valid    (ldst_ld_exec_valid),
        .ld_exec_idx      (ldst_ld_exec_idx),
        .ld_exec_addr     (ldst_ld_exec_addr),
        .st_exec_valid    (ldst_st_exec_valid),
        .st_exec_idx      (ldst_st_exec_idx),
        .st_exec_addr     (ldst_st_exec_addr),
        .st_exec_data     (ldst_st_exec_data),
        .complete         (ldst_complete),
        .rob_idx_out      (ldst_rob_idx_out),
        .exception        (ldst_exception),
        .exc_cause        (ldst_exc_cause),
        .mispredicted_out (),
        .branch_target_out()
    );

    // LSQ
    lsq #(
        .LDQ_ENTRIES(LDQ_ENTRIES),
        .STQ_ENTRIES(STQ_ENTRIES),
        .DATA_W     (DATA_W),
        .ROB_IDX_W  (ROB_IDX_W),
        .PHYS_TAG_W (PHYS_TAG_W),
        .ISSUE_WIDTH(ISSUE_WIDTH)
    ) u_lsq (
        .clk                 (clk),
        .rst_n               (rst_n),
        .dispatch_en         (lsq_dispatch_en),
        .dispatch_is_load    (lsq_dispatch_is_load),
        .dispatch_is_store   (lsq_dispatch_is_store),
        .dispatch_rob_idx    (lsq_dispatch_rob_idx),
        .dispatch_phys_dst   (lsq_dispatch_phys_dst),
        .dispatch_mem_size   (lsq_dispatch_mem_size),
        .dispatch_sign_ext   (lsq_dispatch_sign_ext),
        .ldq_idx_out         (lsq_ldq_idx),
        .stq_idx_out         (lsq_stq_idx),
        .ld_exec_valid       (ldst_ld_exec_valid),
        .ld_exec_idx         (ldst_ld_exec_idx),
        .ld_exec_addr        (ldst_ld_exec_addr),
        .st_exec_valid       (ldst_st_exec_valid),
        .st_exec_idx         (ldst_st_exec_idx),
        .st_exec_addr        (ldst_st_exec_addr),
        .st_exec_data        (ldst_st_exec_data),
        .l1_rd_req           (dcache_rd_req),
        .l1_rd_addr          (dcache_rd_addr),
        .l1_rd_valid         (dcache_rd_valid),
        .l1_rd_data          (dcache_rd_data),
        .l1_wr_req           (dcache_wr_req),
        .l1_wr_addr          (dcache_wr_addr),
        .l1_wr_data          (dcache_wr_data),
        .l1_wr_size          (dcache_wr_size),
        .commit_is_load      (commit_is_load),
        .commit_is_store     (commit_is_store),
        .commit_ldq_idx      (commit_lsq_idx),
        .commit_stq_idx      (commit_lsq_idx),
        .ld_result_valid     (lsq_ld_result_valid),
        .ld_result_tag       (lsq_ld_result_tag),
        .ld_result_data      (lsq_ld_result_data),
        .ld_result_rob_idx   (lsq_ld_result_rob_idx),
        .ld_replay           (lsq_ld_replay),
        .mem_order_violation (lsq_mem_order_violation),
        .violation_rob_idx   (lsq_violation_rob_idx),
        .flush               (flush),
        .ldq_full            (ldq_full),
        .stq_full            (stq_full)
    );

    // CSR
    csr #(.DATA_W(DATA_W)) u_csr (
        .clk           (clk),
        .rst_n         (rst_n),
        .flush         (flush),
        .exc_commit_en (commit_exception),
        .exc_pc_in     (commit_pc[0]),
        .exc_cause_in  (commit_exc_cause),
        .exc_val_in    ('0),
        .mret_en       (mret_en),
        .mret_pc_out   (csr_mret_pc_out),
        .mtvec_out     (csr_mtvec_out),
        .mepc_out      (csr_mepc_out),
        .mcause_out    (csr_mcause_out),
        .commit_count  ({1'b0, commit_en[0]} + {1'b0, commit_en[1]}),
        .mie           (csr_mie),
        .mpie          (csr_mpie)
    );

endmodule