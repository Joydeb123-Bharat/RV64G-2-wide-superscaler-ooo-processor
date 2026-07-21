`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 15.07.2026 20:42:57
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: issue_queue
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: 16-entry compacting Issue Queue (IQ). Accepts up to ISSUE_WIDTH (2)
//              dispatches per cycle. Monitors N_EXEC_UNITS (4) wakeup tag buses each
//              cycle and sets src_rdy bits when a match is found. Each cycle selects
//              the oldest ready entry per execution unit (ALU0, ALU1, MULDIV, LDST)
//              and issues it. Compacts entries upward on issue. Flushes all entries
//              on global flush. Stores branch predictor snapshot (predicted_taken,
//              predicted_target, ghr_snapshot) per entry for branch resolution.
//
// Dependencies: (none — internal CAM/FIFO)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Unit select encoding: 2'b00=ALU (may use ALU0 or ALU1), 2'b01=MULDIV,
//   2'b10=LDST. Branches/JAL/JALR are dispatched with UNIT_ALU and routed to ALU0
//   slot (issue_valid[0]) which feeds the branch_unit combinationally.
//
//////////////////////////////////////////////////////////////////////////////////

module issue_queue #(
    parameter IQ_ENTRIES   = 16,
    parameter PHYS_TAG_W   = 7,
    parameter ISSUE_WIDTH  = 2,
    parameter ROB_IDX_W    = 6,
    parameter DATA_W       = 64,
    parameter N_EXEC_UNITS = 4,
    parameter GHR_W        = 12
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // dispatch interface
    input  logic [ISSUE_WIDTH-1:0]   dispatch_en,
    input  logic [PHYS_TAG_W-1:0]    dispatch_src1          [0:ISSUE_WIDTH-1],
    input  logic [PHYS_TAG_W-1:0]    dispatch_src2          [0:ISSUE_WIDTH-1],
    input  logic [PHYS_TAG_W-1:0]    dispatch_dst           [0:ISSUE_WIDTH-1],
    input  logic [ISSUE_WIDTH-1:0]   dispatch_src1_rdy,
    input  logic [ISSUE_WIDTH-1:0]   dispatch_src2_rdy,
    input  logic [ROB_IDX_W-1:0]     dispatch_rob_idx       [0:ISSUE_WIDTH-1],
    input  logic [3:0]               dispatch_op            [0:ISSUE_WIDTH-1],
    input  logic [DATA_W-1:0]        dispatch_imm           [0:ISSUE_WIDTH-1],
    input  logic [DATA_W-1:0]        dispatch_pc            [0:ISSUE_WIDTH-1],
    input  logic [2:0]               dispatch_funct3        [0:ISSUE_WIDTH-1],  // funct3 for branch and MULDIV
    input  logic [ISSUE_WIDTH-1:0]   dispatch_uses_imm,
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_load,
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_store,
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_branch,
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_jal,                           // JAL flag
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_jalr,                          // JALR flag
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_mul,                           // MUL flag
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_div,                           // DIV flag
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_word,                          // word operation
    input  logic [1:0]               dispatch_mem_size      [0:ISSUE_WIDTH-1],  // memory access size
    input  logic [4:0]               dispatch_lsq_idx       [0:ISSUE_WIDTH-1],  // LSQ slot allocated
    input  logic [ISSUE_WIDTH-1:0]   dispatch_predicted_taken,
    input  logic [DATA_W-1:0]        dispatch_predicted_target [0:ISSUE_WIDTH-1],
    input  logic [GHR_W-1:0]         dispatch_ghr_snapshot  [0:ISSUE_WIDTH-1],  // GHR at dispatch
    input  logic [1:0]               dispatch_unit_sel      [0:ISSUE_WIDTH-1],

    // wakeup interface
    input  logic [N_EXEC_UNITS-1:0]  wakeup_valid,
    input  logic [PHYS_TAG_W-1:0]    wakeup_tag             [0:N_EXEC_UNITS-1],

    // unit availability
    input  logic [N_EXEC_UNITS-1:0]  unit_available,

    // issue interface
    output logic [N_EXEC_UNITS-1:0]  issue_valid,
    output logic [PHYS_TAG_W-1:0]    issue_src1             [0:N_EXEC_UNITS-1],
    output logic [PHYS_TAG_W-1:0]    issue_src2             [0:N_EXEC_UNITS-1],
    output logic [PHYS_TAG_W-1:0]    issue_dst              [0:N_EXEC_UNITS-1],
    output logic [ROB_IDX_W-1:0]     issue_rob_idx          [0:N_EXEC_UNITS-1],
    output logic [3:0]               issue_op               [0:N_EXEC_UNITS-1],
    output logic [DATA_W-1:0]        issue_imm              [0:N_EXEC_UNITS-1],
    output logic [DATA_W-1:0]        issue_pc               [0:N_EXEC_UNITS-1],
    output logic [2:0]               issue_funct3           [0:N_EXEC_UNITS-1], // funct3 to exec unit
    output logic [N_EXEC_UNITS-1:0]  issue_uses_imm,
    output logic [N_EXEC_UNITS-1:0]  issue_is_load,
    output logic [N_EXEC_UNITS-1:0]  issue_is_store,
    output logic [N_EXEC_UNITS-1:0]  issue_is_branch,
    output logic [N_EXEC_UNITS-1:0]  issue_is_jal,                              // JAL to exec unit
    output logic [N_EXEC_UNITS-1:0]  issue_is_jalr,                             // JALR to exec unit
    output logic [N_EXEC_UNITS-1:0]  issue_is_mul,                              // MUL to exec unit
    output logic [N_EXEC_UNITS-1:0]  issue_is_div,                              // DIV to exec unit
    output logic [N_EXEC_UNITS-1:0]  issue_is_word,                             // word op to exec unit
    output logic [1:0]               issue_mem_size         [0:N_EXEC_UNITS-1], // mem size to exec unit
    output logic [4:0]               issue_lsq_idx          [0:N_EXEC_UNITS-1], // LSQ slot to exec unit
    output logic [N_EXEC_UNITS-1:0]  issue_predicted_taken,
    output logic [DATA_W-1:0]        issue_predicted_target [0:N_EXEC_UNITS-1],
    output logic [GHR_W-1:0]         issue_ghr_snapshot     [0:N_EXEC_UNITS-1], // GHR snapshot to exec unit

    // flush
    input  logic                     flush,

    // load replay - when LSQ signals replay, keep the LDST load in IQ for re-issue
    input  logic                     ld_replay,

    // status
    output logic                     iq_full,
    output logic                     iq_empty
);

    localparam IQ_IDX_W = $clog2(IQ_ENTRIES);

    typedef struct packed {
        logic [PHYS_TAG_W-1:0]  src1;
        logic [PHYS_TAG_W-1:0]  src2;
        logic [PHYS_TAG_W-1:0]  dst;
        logic [ROB_IDX_W-1:0]   rob_idx;
        logic [3:0]             op;
        logic [DATA_W-1:0]      imm;
        logic [DATA_W-1:0]      pc;
        logic [DATA_W-1:0]      predicted_target;
        logic [GHR_W-1:0]       ghr_snapshot;     // GHR saved at dispatch for PHT update
        logic [2:0]             funct3;           // funct3 for branch and MULDIV
        logic [1:0]             unit_sel;
        logic [1:0]             mem_size;         // memory access size
        logic [4:0]             lsq_idx;          // LSQ slot allocated at dispatch
        logic                   src1_rdy;
        logic                   src2_rdy;
        logic                   uses_imm;
        logic                   is_load;
        logic                   is_store;
        logic                   is_branch;
        logic                   is_jal;           // JAL instruction
        logic                   is_jalr;          // JALR instruction
        logic                   is_mul;           // MUL instruction
        logic                   is_div;           // DIV instruction
        logic                   is_word;          // word operation
        logic                   predicted_taken;
        logic                   valid;
    } iq_entry_t;

    iq_entry_t               iq              [0:IQ_ENTRIES-1];
    logic                    wakeup_src1_hit [0:IQ_ENTRIES-1];
    logic                    wakeup_src2_hit [0:IQ_ENTRIES-1];
    logic                    entry_ready     [0:IQ_ENTRIES-1];
    logic                    grant           [0:IQ_ENTRIES-1];
    logic [IQ_IDX_W-1:0]     grant_idx       [0:N_EXEC_UNITS-1];
    logic [N_EXEC_UNITS-1:0] grant_valid;
    logic [IQ_IDX_W:0]       iq_count;
    logic [IQ_IDX_W:0]       grant_count;
    logic [IQ_IDX_W:0]       eff_grant_count; // grant_count minus retained load
    logic [IQ_IDX_W:0]       iq_count_next;
    logic [1:0]              dispatch_count;
    logic [IQ_IDX_W-1:0]     dispatch_addr   [0:ISSUE_WIDTH-1];
    logic [2:0]              grants_below    [0:IQ_ENTRIES-1];
    logic                    keep_ldst;      // 1 when LDST load must stay in IQ (replay)

    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            for(int i = 0; i < IQ_ENTRIES; i++)
            begin
                iq[i].src1             <= '0;
                iq[i].src2             <= '0;
                iq[i].dst              <= '0;
                iq[i].rob_idx          <= '0;
                iq[i].op               <= '0;
                iq[i].imm              <= '0;
                iq[i].pc               <= '0;
                iq[i].predicted_target <= '0;
                iq[i].ghr_snapshot     <= '0;
                iq[i].funct3           <= '0;
                iq[i].unit_sel         <= '0;
                iq[i].mem_size         <= '0;
                iq[i].lsq_idx          <= '0;
                iq[i].src1_rdy         <= '0;
                iq[i].src2_rdy         <= '0;
                iq[i].uses_imm         <= '0;
                iq[i].is_load          <= '0;
                iq[i].is_store         <= '0;
                iq[i].is_branch        <= '0;
                iq[i].is_jal           <= '0;
                iq[i].is_jalr          <= '0;
                iq[i].is_mul           <= '0;
                iq[i].is_div           <= '0;
                iq[i].is_word          <= '0;
                iq[i].predicted_taken  <= '0;
                iq[i].valid            <= '0;
            end
            iq_count <= '0;
        end
        else
        begin
            if(flush)
            begin
                for(int i = 0; i < IQ_ENTRIES; i++)
                    iq[i].valid <= '0;
                iq_count <= '0;
            end
            else
            begin
                for(int i = 0; i < IQ_ENTRIES; i++)
                begin: WAKEUP_UPDATE
                    // keep entry if not granted, or if it's the LDST load being replayed
                    if((!grant[i] || (keep_ldst && IQ_IDX_W'(i) == grant_idx[3])) && iq[i].valid)
                    begin
                        iq[IQ_IDX_W'(i - grants_below[i])]          <= iq[i];
                        iq[IQ_IDX_W'(i - grants_below[i])].src1_rdy <= iq[i].src1_rdy | wakeup_src1_hit[i];
                        iq[IQ_IDX_W'(i - grants_below[i])].src2_rdy <= iq[i].src2_rdy | wakeup_src2_hit[i];
                    end
                end

                for(int i = 0; i < IQ_ENTRIES; i++)
                begin
                    if(IQ_IDX_W'(i) >= IQ_IDX_W'(iq_count_next))
                        iq[i].valid <= 1'b0;
                end

                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin: DISPATCH_ASSIGNMENT
                    if(dispatch_en[i])
                    begin
                        iq[dispatch_addr[i]].src1             <= dispatch_src1[i];
                        iq[dispatch_addr[i]].src2             <= dispatch_src2[i];
                        iq[dispatch_addr[i]].dst              <= dispatch_dst[i];
                        iq[dispatch_addr[i]].rob_idx          <= dispatch_rob_idx[i];
                        iq[dispatch_addr[i]].op               <= dispatch_op[i];
                        iq[dispatch_addr[i]].imm              <= dispatch_imm[i];
                        iq[dispatch_addr[i]].pc               <= dispatch_pc[i];
                        iq[dispatch_addr[i]].predicted_target <= dispatch_predicted_target[i];
                        iq[dispatch_addr[i]].ghr_snapshot     <= dispatch_ghr_snapshot[i];
                        iq[dispatch_addr[i]].funct3           <= dispatch_funct3[i];
                        iq[dispatch_addr[i]].unit_sel         <= dispatch_unit_sel[i];
                        iq[dispatch_addr[i]].mem_size         <= dispatch_mem_size[i];
                        iq[dispatch_addr[i]].lsq_idx          <= dispatch_lsq_idx[i];
                        iq[dispatch_addr[i]].src1_rdy         <= dispatch_src1_rdy[i];
                        iq[dispatch_addr[i]].src2_rdy         <= dispatch_src2_rdy[i];
                        iq[dispatch_addr[i]].uses_imm         <= dispatch_uses_imm[i];
                        iq[dispatch_addr[i]].is_load          <= dispatch_is_load[i];
                        iq[dispatch_addr[i]].is_store         <= dispatch_is_store[i];
                        iq[dispatch_addr[i]].is_branch        <= dispatch_is_branch[i];
                        iq[dispatch_addr[i]].is_jal           <= dispatch_is_jal[i];
                        iq[dispatch_addr[i]].is_jalr          <= dispatch_is_jalr[i];
                        iq[dispatch_addr[i]].is_mul           <= dispatch_is_mul[i];
                        iq[dispatch_addr[i]].is_div           <= dispatch_is_div[i];
                        iq[dispatch_addr[i]].is_word          <= dispatch_is_word[i];
                        iq[dispatch_addr[i]].predicted_taken  <= dispatch_predicted_taken[i];
                        iq[dispatch_addr[i]].valid            <= 1'b1;
                    end
                end
                iq_count <= iq_count_next;
            end
        end
    end

    always_comb
    begin
        // Step 1: compute wakeup hits and entry_ready
        for(int i = 0; i < IQ_ENTRIES; i++)
        begin
            wakeup_src1_hit[i] = '0;
            wakeup_src2_hit[i] = '0;
            if(iq[i].valid)
            begin
                for(int b = 0; b < N_EXEC_UNITS; b++)
                begin
                    if(wakeup_valid[b] && (!iq[i].src1_rdy) && (wakeup_tag[b] == iq[i].src1))
                        wakeup_src1_hit[i] = 1'b1;
                    if(wakeup_valid[b] && (!iq[i].src2_rdy) && (wakeup_tag[b] == iq[i].src2))
                        wakeup_src2_hit[i] = 1'b1;
                end
            end
            entry_ready[i] = (iq[i].src1_rdy | wakeup_src1_hit[i]) &
                             (iq[i].src2_rdy | wakeup_src2_hit[i] | (iq[i].uses_imm & !iq[i].is_store)) &
                             iq[i].valid;
        end

        // Step 2: compute grants (must come before dispatch_addr so grant_count is known)
        for(int i = 0; i < IQ_ENTRIES; i++)
            grant[i] = 1'b0;
        for(int u = 0; u < N_EXEC_UNITS; u++)
        begin
            grant_idx[u]   = '0;
            grant_valid[u] = 1'b0;
        end

        // Branch priority pre-pass: scan oldest-first for branch/JAL/JALR
        // and give it grant[0] so branch_unit (wired to slot 0) always fires.
        for (int i = 0; i < IQ_ENTRIES; i++) begin
            if (entry_ready[i] && iq[i].valid && iq[i].unit_sel == 2'b00
                && (iq[i].is_branch | iq[i].is_jal | iq[i].is_jalr)
                && !grant_valid[0] && unit_available[0]) begin
                grant_valid[0] = 1'b1;
                grant_idx[0]   = IQ_IDX_W'(i);
                grant[i]       = 1'b1;
            end
            if (grant_valid[0]) break;
        end

        for(int i = 0; i < IQ_ENTRIES; i++)
        begin
            if(entry_ready[i] && iq[i].valid)
            begin
                if(iq[i].unit_sel == 2'b00 && !grant_valid[0] && unit_available[0])
                begin
                    grant_valid[0] = 1'b1;
                    grant_idx[0]   = IQ_IDX_W'(i);
                    grant[i]       = 1'b1;
                end
                else if(iq[i].unit_sel == 2'b00 && !grant_valid[1] && unit_available[1] &&
                        !(grant_valid[0] && grant_idx[0] == IQ_IDX_W'(i)))
                begin
                    grant_valid[1] = 1'b1;
                    grant_idx[1]   = IQ_IDX_W'(i);
                    grant[i]       = 1'b1;
                end
                else if(iq[i].unit_sel == 2'b01 && !grant_valid[2] && unit_available[2])
                begin
                    grant_valid[2] = 1'b1;
                    grant_idx[2]   = IQ_IDX_W'(i);
                    grant[i]       = 1'b1;
                end
                else if(iq[i].unit_sel == 2'b10 && !grant_valid[3] && unit_available[3])
                begin
                    grant_valid[3] = 1'b1;
                    grant_idx[3]   = IQ_IDX_W'(i);
                    grant[i]       = 1'b1;
                end

                if(grant_valid == 4'b1111)
                    break;
            end
        end

        // Step 3: now that grant_count is known, compute dispatch addresses relative
        // to the compacted tail (iq_count - eff_grant_count), so issued slots are reused.
        grant_count = $countones(grant_valid);
        // keep_ldst: LDST load stays in IQ when LSQ signals replay
        keep_ldst   = ld_replay & grant_valid[3] & iq[grant_idx[3]].is_load;
        eff_grant_count = grant_count - {4'd0, keep_ldst};
        dispatch_count = '0;
        begin: DISPATCH_ADDR_BLOCK
            logic [IQ_IDX_W-1:0] offset;
            offset = '0;
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                dispatch_addr[i] = IQ_IDX_W'((iq_count - eff_grant_count) + offset);
                if(dispatch_en[i])
                begin
                    offset         = offset + 1'd1;
                    dispatch_count += 1'd1;
                end
            end
        end

        iq_count_next = (iq_count - eff_grant_count) + dispatch_count;
        iq_full       = (iq_count > IQ_IDX_W'(IQ_ENTRIES - ISSUE_WIDTH));
        iq_empty      = (iq_count == '0);

        for(int i = 0; i < N_EXEC_UNITS; i++)
        begin
            issue_valid[i]            = '0;
            issue_src1[i]             = '0;
            issue_src2[i]             = '0;
            issue_dst[i]              = '0;
            issue_rob_idx[i]          = '0;
            issue_op[i]               = '0;
            issue_imm[i]              = '0;
            issue_pc[i]               = '0;
            issue_funct3[i]           = '0;
            issue_uses_imm[i]         = '0;
            issue_is_load[i]          = '0;
            issue_is_store[i]         = '0;
            issue_is_branch[i]        = '0;
            issue_is_jal[i]           = '0;
            issue_is_jalr[i]          = '0;
            issue_is_mul[i]           = '0;
            issue_is_div[i]           = '0;
            issue_is_word[i]          = '0;
            issue_mem_size[i]         = '0;
            issue_lsq_idx[i]          = '0;
            issue_predicted_taken[i]  = '0;
            issue_predicted_target[i] = '0;
            issue_ghr_snapshot[i]     = '0;

            if(grant_valid[i])
            begin
                issue_valid[i]            = iq[grant_idx[i]].valid;
                issue_src1[i]             = iq[grant_idx[i]].src1;
                issue_src2[i]             = iq[grant_idx[i]].src2;
                issue_dst[i]              = iq[grant_idx[i]].dst;
                issue_rob_idx[i]          = iq[grant_idx[i]].rob_idx;
                issue_op[i]               = iq[grant_idx[i]].op;
                issue_imm[i]              = iq[grant_idx[i]].imm;
                issue_pc[i]               = iq[grant_idx[i]].pc;
                issue_funct3[i]           = iq[grant_idx[i]].funct3;
                issue_uses_imm[i]         = iq[grant_idx[i]].uses_imm;
                issue_is_load[i]          = iq[grant_idx[i]].is_load;
                issue_is_store[i]         = iq[grant_idx[i]].is_store;
                issue_is_branch[i]        = iq[grant_idx[i]].is_branch;
                issue_is_jal[i]           = iq[grant_idx[i]].is_jal;
                issue_is_jalr[i]          = iq[grant_idx[i]].is_jalr;
                issue_is_mul[i]           = iq[grant_idx[i]].is_mul;
                issue_is_div[i]           = iq[grant_idx[i]].is_div;
                issue_is_word[i]          = iq[grant_idx[i]].is_word;
                issue_mem_size[i]         = iq[grant_idx[i]].mem_size;
                issue_lsq_idx[i]          = iq[grant_idx[i]].lsq_idx;
                issue_predicted_taken[i]  = iq[grant_idx[i]].predicted_taken;
                issue_predicted_target[i] = iq[grant_idx[i]].predicted_target;
                issue_ghr_snapshot[i]     = iq[grant_idx[i]].ghr_snapshot;
            end
        end

        for(int i = 0; i < IQ_ENTRIES; i++)
        begin
            grants_below[i] = 3'd0;
            for(int u = 0; u < N_EXEC_UNITS; u++)
            begin
                // don't count LDST slot as a compaction shift when it's being retained
                if(grant_valid[u] && (grant_idx[u] < IQ_IDX_W'(i)) && !(keep_ldst && u == 3))
                    grants_below[i] = grants_below[i] + 3'd1;
            end
        end
    end

endmodule