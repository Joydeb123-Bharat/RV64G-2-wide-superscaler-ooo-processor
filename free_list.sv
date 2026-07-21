`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 14.07.2026 21:24:35
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: free_list
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: FIFO-based physical register free list. At reset, loads
//              N_PHYS_REGS - N_ARCH_REGS = 64 free tags (physical tags 32–95)
//              since tags 0–31 are identity-mapped at startup. Supports up to
//              ISSUE_WIDTH (2) pops per cycle at rename and ISSUE_WIDTH pushes
//              per cycle at commit (returning the stale tag). Saves a checkpoint
//              of the FIFO head pointer at every branch dispatch (indexed by
//              ROB slot) and restores it on flush for misprediction recovery.
//              Asserts stall when available count < ISSUE_WIDTH.
//
// Dependencies: (none — FIFO with checkpoint array)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   PTR_W = FIFO_DEPTH_LOG+1 uses an extra MSB for full/empty disambiguation
//   (standard FIFO pointer trick). N_CHECKPOINTS = ROB_ENTRIES = 64.
//
//////////////////////////////////////////////////////////////////////////////////

module free_list #(
    parameter N_PHYS_REGS  = 96,   // total physical registers
    parameter N_ARCH_REGS  = 32,   // architectural registers (initially mapped)
    parameter ISSUE_WIDTH  = 2,    // max pops and pushes per cycle
    parameter PHYS_TAG_W   = 7,    // ceil(log2(N_PHYS_REGS))
    parameter ROB_ENTRIES  = 64,   // number of ROB slots for checkpoint indexing
    parameter ROB_IDX_W    = 6     // ceil(log2(ROB_ENTRIES))
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // pop interface - rename/dispatch side
    input  logic [ISSUE_WIDTH-1:0]   pop_req,
    output logic [PHYS_TAG_W-1:0]    pop_tag  [0:ISSUE_WIDTH-1],
    output logic [ISSUE_WIDTH-1:0]   pop_valid,

    // push interface - commit side
    input  logic [ISSUE_WIDTH-1:0]   push_req,
    input  logic [PHYS_TAG_W-1:0]    push_tag [0:ISSUE_WIDTH-1],

    // checkpoint interface - snapshot head at every branch dispatch
    input  logic                     ckpt_en,
    input  logic [ROB_IDX_W-1:0]    ckpt_idx,
    // ckpt_offset: number of pops to skip before saving head (1 for JAL/JALR, 0 for branches)
    // Ensures JAL/JALR's own rd allocation is NOT restored on flush (it is committed).
    input  logic [1:0]               ckpt_offset,

    // flush interface - restore head from checkpoint on misprediction
    input  logic                     flush,
    input  logic [ROB_IDX_W-1:0]    flush_rob_idx,

    // status
    output logic [PHYS_TAG_W:0]      fl_count, // number of free tags currently available
    output logic                     stall      // high when count < ISSUE_WIDTH
);

    // local parameters
    localparam FIFO_DEPTH     = N_PHYS_REGS;           // FIFO sized to cover all physical regs
    localparam FIFO_DEPTH_LOG = $clog2(FIFO_DEPTH);    // index bit width into fifo_mem
    localparam PTR_W          = FIFO_DEPTH_LOG + 1;    // extra MSB for full/empty disambiguation
    localparam N_CHECKPOINTS  = ROB_ENTRIES;            // one head snapshot per ROB slot
    localparam INIT_FREE_CNT  = N_PHYS_REGS - N_ARCH_REGS; // tags loaded at reset = 64

    // FIFO storage and pointers
    logic [PHYS_TAG_W-1:0]      fifo_mem [0:FIFO_DEPTH-1];   // circular buffer holding free tags
    logic [PTR_W-1:0]           head;                        // pop-side pointer
    logic [PTR_W-1:0]           tail;                        // push-side pointer
    logic                       empty, full;                 // Status signals
    // checkpoint storage
    logic [PTR_W-1:0]           head_ckpt [0:N_CHECKPOINTS-1]; // head snapshots indexed by ROB slot

    // derived index signals - strip wrap MSB for array indexing
    logic [FIFO_DEPTH_LOG-1:0]  head_idx; // lower bits of head used to index fifo_mem
    logic [FIFO_DEPTH_LOG-1:0]  tail_idx; // lower bits of tail used to index fifo_mem

    // pop and push counts - how many slots are active this cycle
    logic [1:0]                 pop_count;  // number of asserted bits in pop_req
    logic [1:0]                 push_count; // number of asserted bits in push_req
    logic [1:0]                 push_offset; // running offset for push writes into fifo_mem
    logic [FIFO_DEPTH_LOG-1:0]  push_addr [0:ISSUE_WIDTH-1]; // computed push addresses
    
    // 1. Sequential Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
           for(int i = 0; i < FIFO_DEPTH; i++)
           begin: FIFO_Default
                fifo_mem[FIFO_DEPTH_LOG'(i)] <= (i < INIT_FREE_CNT) ? PHYS_TAG_W'(32 + i) : '0;
           end  
           for(int i = 0; i < N_CHECKPOINTS; i++)
           begin: CKPT
                head_ckpt[ROB_IDX_W'(i)] <= '0;
           end
           head <= '0;
           tail <= PTR_W'(INIT_FREE_CNT);
           
        end
        else 
        begin
            // POP logic
            if(flush)
            begin
                head <= head_ckpt[flush_rob_idx];
            end 
            else
            begin
                head <= head + PTR_W'(pop_count);
                if(ckpt_en)
                head_ckpt[ckpt_idx] <= head + PTR_W'(ckpt_offset);
            end
            
            // PUSH logic
            if(push_req != '0)
            begin
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(push_req[i])
                    begin
                        fifo_mem[push_addr[i]] <= push_tag[i];
                    end
                end
                tail <= tail + push_count;
            end
        end
    end
    
    // 2. Combinational block
    always_comb
    begin
        head_idx = head[PTR_W-2:0];
        tail_idx = tail[PTR_W-2:0];
        empty = head == tail;
        full = (head_idx == tail_idx) && (head[PTR_W-1] != tail[PTR_W-1]);
        fl_count = tail - head;
        stall = fl_count < PHYS_TAG_W'(ISSUE_WIDTH);
        
        // Default values
        pop_count = '0;
        push_count = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            pop_valid[i] = 1'b0;
            pop_tag[i] = '0;
        end
        push_offset = 2'd0;
        
        // POP logic
        if(flush)
        begin
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                pop_valid[i] = 1'b0;
            end
            pop_count = '0;
        end
        else
        begin
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                if(pop_req[i] && (fl_count > PHYS_TAG_W'(pop_count)))
                begin
                    pop_tag[i] = fifo_mem[FIFO_DEPTH_LOG'(head_idx + pop_count)];
                    pop_count += 2'd1;
                    pop_valid[i] = 1'b1;
                end
            end
        end
        
        // PUSH logic
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            push_addr[i] = FIFO_DEPTH_LOG'(tail_idx + push_offset);
            if(push_req[i])
            begin
                push_offset = push_offset + 2'd1;
            end
            push_count += push_req[i];
        end
        
    end
endmodule