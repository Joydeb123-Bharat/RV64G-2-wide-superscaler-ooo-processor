`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 15.07.2026 14:45:10
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: rob
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: 64-entry circular Reorder Buffer (ROB). Accepts up to ISSUE_WIDTH (2)
//              instruction dispatches per cycle at TAIL and commits up to ISSUE_WIDTH
//              instructions per cycle in program order from HEAD. Tracks execution
//              completion, exception flags, misprediction flags, and branch targets
//              per entry. On misprediction or exception at HEAD, asserts flush,
//              flush_target (correct PC), and flush_rob_idx for checkpoint recovery.
//              Drives the commit interface consumed by Retire RAT, Free List, and LSQ.
//
// Dependencies: (none — internal circular buffer)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   ROB_IDX_W=6 for 64 entries. Exception redirect uses CSR mtvec; misprediction
//   redirect uses the resolved branch target stored in the ROB entry.
//   LSQ_ENTRIES parameter sizes the lsq_idx field stored per entry.
//
//////////////////////////////////////////////////////////////////////////////////

module rob #(
    parameter ROB_ENTRIES  = 64,
    parameter N_ARCH_REGS  = 32,
    parameter PHYS_TAG_W   = 7,
    parameter ISSUE_WIDTH  = 2,
    parameter DATA_W       = 64,
    parameter LSQ_ENTRIES  = 16
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // dispatch interface - written by rename/dispatch stage each cycle
    input  logic [ISSUE_WIDTH-1:0]    dispatch_en,
    input  logic [DATA_W-1:0]         dispatch_pc        [0:ISSUE_WIDTH-1],
    input  logic [4:0]                dispatch_arch_rd   [0:ISSUE_WIDTH-1],
    input  logic [PHYS_TAG_W-1:0]     dispatch_phys_rd   [0:ISSUE_WIDTH-1],
    input  logic [PHYS_TAG_W-1:0]     dispatch_stale     [0:ISSUE_WIDTH-1],
    input  logic [4:0]                dispatch_lsq_idx   [0:ISSUE_WIDTH-1],
    input  logic [ISSUE_WIDTH-1:0]    dispatch_is_store,
    input  logic [ISSUE_WIDTH-1:0]    dispatch_is_load,
    input  logic [ISSUE_WIDTH-1:0]    dispatch_is_branch,
    input  logic [ISSUE_WIDTH-1:0]    dispatch_has_rd,

    // execute complete interface - written by execution units on completion
    input  logic [ISSUE_WIDTH-1:0]    exec_complete,
    input  logic [5:0]                exec_rob_idx       [0:ISSUE_WIDTH-1],
    input  logic [ISSUE_WIDTH-1:0]    exec_exception,
    input  logic [3:0]                exec_exc_cause     [0:ISSUE_WIDTH-1],
    input  logic [ISSUE_WIDTH-1:0]    exec_mispredicted,
    input  logic [DATA_W-1:0]         exec_branch_target [0:ISSUE_WIDTH-1],

    // CSR interface - trap vector for exception redirect
    input  logic [DATA_W-1:0]         trap_handler_pc,

    // commit interface - consumed by Retire RAT, Free List, STQ, LDQ
    output logic [ISSUE_WIDTH-1:0]    commit_en,
    output logic [4:0]                commit_arch_rd     [0:ISSUE_WIDTH-1],
    output logic [PHYS_TAG_W-1:0]     commit_phys_rd     [0:ISSUE_WIDTH-1],
    output logic [PHYS_TAG_W-1:0]     commit_stale       [0:ISSUE_WIDTH-1],
    output logic [ISSUE_WIDTH-1:0]    commit_is_store,
    output logic [ISSUE_WIDTH-1:0]    commit_is_load,
    output logic [ISSUE_WIDTH-1:0]    commit_has_rd,
    output logic [4:0]                commit_lsq_idx     [0:ISSUE_WIDTH-1],
    output logic [DATA_W-1:0]         commit_pc          [0:ISSUE_WIDTH-1],
    output logic                      commit_exception,
    output logic [3:0]                commit_exc_cause,

    // flush interface - broadcast to every pipeline stage simultaneously
    output logic                      flush,
    output logic [DATA_W-1:0]         flush_target,
    output logic [5:0]                flush_rob_idx,

    // ROB index outputs - combinational, visible to dispatch before clock edge
    output logic [5:0]                rob_idx [0:ISSUE_WIDTH-1],

    // status
    output logic                      rob_full,
    output logic                      rob_empty
);

    // local parameters
    localparam ROB_IDX_W   = $clog2(ROB_ENTRIES);
    localparam PTR_W       = ROB_IDX_W + 1;
    localparam LSQ_IDX_W   = $clog2(LSQ_ENTRIES);

    // packed struct for internal storage only - never used on ports
    typedef struct packed {
        logic [DATA_W-1:0]      pc;
        logic [DATA_W-1:0]      branch_target;
        logic [4:0]             arch_rd;
        logic [PHYS_TAG_W-1:0]  phys_rd;
        logic [PHYS_TAG_W-1:0]  stale_phys;
        logic [4:0]             lsq_idx;
        logic [3:0]             exc_cause;
        logic                   complete;
        logic                   exception;
        logic                   mispredicted;
        logic                   is_store;
        logic                   is_load;
        logic                   is_branch;
        logic                   has_rd;
        logic                   valid;
    } rob_entry_t;

    // ROB storage - one struct entry per slot
    rob_entry_t              rob          [0:ROB_ENTRIES-1];

    // head and tail pointers
    logic [PTR_W-1:0]        head;
    logic [PTR_W-1:0]        tail;

    // derived index signals
    logic [ROB_IDX_W-1:0]    head_idx;
    logic [ROB_IDX_W-1:0]    tail_idx;

    // dispatch write addresses computed combinationally
    logic [ROB_IDX_W-1:0]    dispatch_addr [0:ISSUE_WIDTH-1];

    // occupancy and flow control
    logic [ROB_IDX_W:0]      free_count;
    logic [1:0]              dispatch_count;
    logic [1:0]              commit_count;

    // 1. Sequential Block
    always_ff @(posedge clk)
    begin
        if(!rst_n)
        begin
            for(int i = 0; i < ROB_ENTRIES; i++)
            begin: ROB_RESET
                rob[i].pc           <= '0;
                rob[i].branch_target <= '0;
                rob[i].arch_rd      <= '0;
                rob[i].phys_rd      <= '0;
                rob[i].stale_phys   <= '0;
                rob[i].lsq_idx      <= '0;
                rob[i].exc_cause    <= '0;
                rob[i].complete     <= 1'b0;
                rob[i].exception    <= 1'b0;
                rob[i].mispredicted <= 1'b0;
                rob[i].is_store     <= 1'b0;
                rob[i].is_load      <= 1'b0;
                rob[i].is_branch    <= 1'b0;
                rob[i].has_rd       <= 1'b0;
                rob[i].valid        <= 1'b0;
            end
            head <= '0;
            tail <= '0;
        end
        else
        begin
            if(!flush)
            begin
                // Dispatch Logic
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(dispatch_en[i] && !rob_full)
                    begin
                        rob[dispatch_addr[i]].pc           <= dispatch_pc[i];
                        rob[dispatch_addr[i]].arch_rd      <= dispatch_arch_rd[i];
                        rob[dispatch_addr[i]].phys_rd      <= dispatch_phys_rd[i];
                        rob[dispatch_addr[i]].stale_phys   <= dispatch_stale[i];
                        rob[dispatch_addr[i]].lsq_idx      <= dispatch_lsq_idx[i];
                        rob[dispatch_addr[i]].is_store     <= dispatch_is_store[i];
                        rob[dispatch_addr[i]].is_load      <= dispatch_is_load[i];
                        rob[dispatch_addr[i]].is_branch    <= dispatch_is_branch[i];
                        rob[dispatch_addr[i]].has_rd       <= dispatch_has_rd[i];
                        rob[dispatch_addr[i]].complete     <= 1'b0;
                        rob[dispatch_addr[i]].exception    <= 1'b0;
                        rob[dispatch_addr[i]].mispredicted <= 1'b0;
                        rob[dispatch_addr[i]].branch_target <= '0;
                        rob[dispatch_addr[i]].exc_cause    <= '0;
                        rob[dispatch_addr[i]].valid        <= 1'b1;
                    end
                end
                tail <= tail + PTR_W'(dispatch_count);

                // Execution Logic
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(exec_complete[i])
                    begin
                        rob[exec_rob_idx[i]].complete      <= 1'b1;
                        rob[exec_rob_idx[i]].exception     <= exec_exception[i];
                        rob[exec_rob_idx[i]].exc_cause     <= exec_exc_cause[i];
                        rob[exec_rob_idx[i]].mispredicted  <= exec_mispredicted[i];
                        rob[exec_rob_idx[i]].branch_target <= exec_branch_target[i];
                    end
                end

                // Commit Logic
                head <= head + PTR_W'(commit_count);
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(commit_en[i])
                    begin
                        rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].valid <= 1'b0;
                    end
                end
            end
            else
            begin
                // Flush Logic
                rob[head_idx].valid    <= 1'b0;
                rob[head_idx].complete <= 1'b0;
                head <= head + PTR_W'(1);
                tail <= head + PTR_W'(1);
            end
        end
    end

    // 2. Combinational Block
    always_comb
    begin
        head_idx   = head[PTR_W-2:0];
        tail_idx   = tail[PTR_W-2:0];
        rob_empty  = (head == tail);
        free_count = PTR_W'(ROB_ENTRIES) - (tail - head);
        rob_full   = (free_count < PTR_W'(ISSUE_WIDTH));

        // dispatch address computation
        begin : DISPATCH_ADDR
            logic [ROB_IDX_W-1:0] offset;
            offset = '0;
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                dispatch_addr[i] = ROB_IDX_W'(tail_idx + offset);
                if(dispatch_en[i])
                begin
                    offset = offset + ROB_IDX_W'(1);
                end
            end
        end

        // dispatch and commit counts
        dispatch_count = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            dispatch_count += dispatch_en[i];
            rob_idx[i] = ROB_IDX_W'(tail_idx + ROB_IDX_W'(i));
        end

        // flush detection
        flush = rob[head_idx].valid & rob[head_idx].complete & (rob[head_idx].exception || rob[head_idx].mispredicted);
        flush_rob_idx = flush ? ROB_IDX_W'(head_idx) : '0;
        flush_target  = flush ? (rob[head_idx].exception ? trap_handler_pc : rob[head_idx].branch_target) : '0;

        // JAL/JALR with return address: commit even on misprediction so RETIRE RAT records the link register
        // Without this, after flush the dispatch RAT restores from RETIRE RAT and x1 stays at the stale tag.
        begin : COMMIT_EN
            logic jal_flush_commit;
            jal_flush_commit = flush & rob[head_idx].has_rd & rob[head_idx].valid
                              & rob[head_idx].complete & !rob[head_idx].exception;
            commit_en[0] = (rob[head_idx].valid & rob[head_idx].complete
                           & !rob[head_idx].exception & !rob[head_idx].mispredicted)
                         | jal_flush_commit;
            // slot 1 only fires on normal (non-flush) dual commit
            commit_en[1] = (rob[head_idx].valid & rob[head_idx].complete
                           & !rob[head_idx].exception & !rob[head_idx].mispredicted)
                         & rob[ROB_IDX_W'(head_idx + 1)].valid
                         & rob[ROB_IDX_W'(head_idx + 1)].complete
                         & !rob[ROB_IDX_W'(head_idx + 1)].exception
                         & !rob[ROB_IDX_W'(head_idx + 1)].mispredicted;
        end

        // commit count after commit_en is resolved
        commit_count = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            commit_count += commit_en[i];
        end

        // commit output defaults
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            commit_arch_rd[i]  = '0;
            commit_phys_rd[i]  = '0;
            commit_stale[i]    = '0;
            commit_is_store[i] = 1'b0;
            commit_is_load[i]  = 1'b0;
            commit_has_rd[i]   = 1'b0;
            commit_lsq_idx[i]  = '0;
            commit_pc[i]       = '0;
        end

        // commit outputs driven from ROB entries
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            if(commit_en[i])
            begin
                commit_arch_rd[i]  = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].arch_rd;
                commit_phys_rd[i]  = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].phys_rd;
                commit_stale[i]    = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].stale_phys;
                commit_is_store[i] = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].is_store;
                commit_is_load[i]  = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].is_load;
                commit_has_rd[i]   = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].has_rd;
                commit_lsq_idx[i]  = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].lsq_idx;
                commit_pc[i]       = rob[ROB_IDX_W'(head_idx + ROB_IDX_W'(i))].pc;
            end
        end

        // exception reporting 
        commit_exception = rob[head_idx].valid & rob[head_idx].complete & rob[head_idx].exception;
        commit_exc_cause = commit_exception ? rob[head_idx].exc_cause : 4'd0;
    end
endmodule