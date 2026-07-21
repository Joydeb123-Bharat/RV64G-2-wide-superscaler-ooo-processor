`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 15.07.2026 11:30:05
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: retire_rat
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Committed (Retire) Register Alias Table. Tracks the last committed
//              architectural-to-physical register mapping. Updated synchronously
//              on each ROB commit: when commit_en[i] is asserted and the committing
//              instruction has a valid destination, the mapping for that architectural
//              register is updated to commit_phys_rd[i]. The full retire_map output
//              is sent to the speculative RAT on every flush to restore correct
//              post-commit state for misprediction and exception recovery.
//
// Dependencies: (none — register array)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Initialised with identity mapping (arch_reg i → phys_tag i) matching the
//   PRF reset state where tags 0–31 hold the initial architectural register values.
//
//////////////////////////////////////////////////////////////////////////////////


module retire_rat #(
    parameter N_ARCH_REGS  = 32,  // number of architectural registers
    parameter N_PHYS_REGS  = 96,  // number of physical registers
    parameter PHYS_TAG_W   = 7,   // ceil(log2(N_PHYS_REGS))
    parameter ISSUE_WIDTH  = 2    // number of instructions committed per cycle
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // commit interface - update mapping when instruction retires
    input  logic [ISSUE_WIDTH-1:0]  commit_en,                     // commit enable per slot
    input  logic [4:0]              commit_arch_rd [0:ISSUE_WIDTH-1], // architectural rd retiring
    input  logic [PHYS_TAG_W-1:0]   commit_phys_rd [0:ISSUE_WIDTH-1], // physical rd retiring

    // flush output - full mapping sent to Front-End RAT on misprediction
    output logic [PHYS_TAG_W-1:0]   retire_map [0:N_ARCH_REGS-1]  // full committed mapping
);

    // local parameters
    localparam ARCH_W = $clog2(N_ARCH_REGS); // bit width to index architectural register array

    // Retire RAT storage
    logic [PHYS_TAG_W-1:0]  map [0:N_ARCH_REGS-1]; // committed arch->phys mapping
    
    // 1. Sequential Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            for(int i = 0; i < N_ARCH_REGS; i++)
            begin
                map[i] <= PHYS_TAG_W'(i);
            end
        end
        else
        begin
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                if(commit_en[i] && (commit_arch_rd[i] != '0))
                    map[commit_arch_rd[i]] <= commit_phys_rd[i];
            end
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        for(int i = 0; i < N_ARCH_REGS; i++)
        begin
            retire_map[i] = map[i];
        end
    end
    
endmodule