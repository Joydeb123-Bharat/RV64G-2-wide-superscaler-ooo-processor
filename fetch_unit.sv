`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 17.07.2026 21:02:22
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: fetch_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: PC management and instruction fetch unit. Fetches ISSUE_WIDTH (2)
//              instruction words per cycle from the I-cache interface. PC resets to
//              0x0000_0000_0000_0000 and advances by 8 bytes per cycle. Handles
//              flush redirects from the ROB (highest priority) and speculative
//              redirects from the branch predictor. A 1-cycle bubble is inserted
//              after any redirect while the redirect_pending flag is set to discard
//              the stale I-cache response.
//
// Dependencies: (none)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   PC is 8-byte aligned at all times (ISSUE_WIDTH=2, each instruction is 4 bytes).
//   icache_pc output always reflects the address sent to cache this cycle.
//
//////////////////////////////////////////////////////////////////////////////////


module fetch_unit #(
    parameter DATA_W      = 64,  // PC and address width
    parameter ISSUE_WIDTH = 2    // instructions fetched per cycle
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // flush interface - highest priority PC redirect
    input  logic                      flush,           // global flush from ROB
    input  logic [DATA_W-1:0]         flush_target,    // correct PC after flush

    // branch predictor interface - speculative redirect
    input  logic                      bp_redirect,     // predictor says take branch this cycle
    input  logic [DATA_W-1:0]         bp_target,       // predicted branch target PC

    // stall interface - back pressure from decode or dispatch
    input  logic                      stall,           // hold PC and re-present instructions

    // I-cache interface
    output logic [DATA_W-1:0]         icache_pc,       // PC sent to I-cache for fetch
    input  logic [31:0]               icache_instr [0:ISSUE_WIDTH-1], // instruction words returned
    input  logic                      icache_valid,    // I-cache has valid instructions this cycle

    // output to decode unit
    output logic [ISSUE_WIDTH-1:0]    fetch_valid,     // valid instruction per slot
    output logic [31:0]               fetch_instr  [0:ISSUE_WIDTH-1], // fetched instruction words
    output logic [DATA_W-1:0]         fetch_pc     [0:ISSUE_WIDTH-1], // PC of each instruction

    // current PC output for branch predictor lookup
    output logic [DATA_W-1:0]         current_pc       // PC being fetched this cycle
);

    // local parameters
    localparam PC_RESET   = 64'h0000_0000_0000_0000; // PC value at reset
    localparam INSTR_SIZE = 3;                        // log2 of bytes per instruction (4 bytes = 2^2, but 3 for 8-byte fetch increment)

    // PC register
    logic [DATA_W-1:0]   pc;            // current program counter
    logic [DATA_W-1:0]   pc_next;       // next PC computed combinationally

    // fetch buffer registers - hold instructions from I-cache
    logic [31:0]            fetch_buf_instr [0:ISSUE_WIDTH-1]; // buffered instruction words
    logic [DATA_W-1:0]      fetch_buf_pc    [0:ISSUE_WIDTH-1]; // buffered instruction PCs
    logic [ISSUE_WIDTH-1:0] fetch_buf_valid;                 // valid bits for buffered instructions
    logic [DATA_W-1:0]      fetch_pc_reg;  // PC that was sent to I-cache last cycle
    
    // redirect tracking
    logic                redirect_pending;  // a redirect was issued last cycle cache response stale
    
    // 1. Sequential Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            pc <= PC_RESET;
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                fetch_buf_instr[i]  <= '0;
                fetch_buf_pc[i]     <= '0;
                fetch_buf_valid[i]  <= '0;
            end
            redirect_pending <= '0;
            fetch_pc_reg <= '0;
        end
        else
        begin
            pc <= pc_next;
            redirect_pending <= flush | bp_redirect;
            fetch_pc_reg <= pc;  // register current PC for use when response arrives next cycle
            if(!redirect_pending && !flush && !bp_redirect && !stall)
            begin
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(icache_valid)
                    begin
                        fetch_buf_instr[i]  <= icache_instr[i];
                        fetch_buf_pc[i]     <= fetch_pc_reg + DATA_W'(4*(i));
                        fetch_buf_valid[i]  <= 1'b1;
                    end
                end
            end
            else
            begin
                fetch_buf_valid[0]  <= 1'b0;
                fetch_buf_valid[1]  <= 1'b0;
            end
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        current_pc = pc;
        icache_pc  = pc;   // drive I-cache address with current PC
        pc_next = pc;
        if(flush)
            pc_next = flush_target;
        else if(bp_redirect)
            pc_next = bp_target;
        else 
        begin
            if(!stall)
                pc_next = pc + DATA_W'(8);
        end
        
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            fetch_valid[i] = fetch_buf_valid[i];
            fetch_pc[i] = fetch_buf_pc[i];
            fetch_instr[i] = fetch_buf_instr[i];
        end
    end
endmodule