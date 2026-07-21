`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 15.07.2026 10:24:54
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: rat
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Speculative Register Alias Table (RAT). Maps 32 architectural
//              registers to physical tags. Provides ISSUE_WIDTH (2) combinational
//              source-register lookup pairs (rs1/rs2 → phys) and ISSUE_WIDTH
//              write ports (rd arch → phys) that update on dispatch. Includes
//              intra-dispatch forwarding: if slot 0 writes to a register that
//              slot 1 reads, the new physical tag is forwarded directly without
//              reading the stale RAT entry. On flush, restores the full mapping
//              from the Retire RAT (committed architectural state).
//
// Dependencies: retire_rat (for flush restore input)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   stale_phys output carries the old physical tag displaced by a rename write;
//   this is stored in the ROB entry and pushed back to the Free List at commit.
//
//////////////////////////////////////////////////////////////////////////////////

module rat #(
    parameter N_ARCH_REGS  = 32,   // number of architectural registers
    parameter N_PHYS_REGS  = 96,   // number of physical registers
    parameter PHYS_TAG_W   = 7,    // ceil(log2(N_PHYS_REGS))
    parameter ISSUE_WIDTH  = 2     // number of instructions renamed per cycle
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // read interface - combinational source register lookup
    input  logic [4:0]              rs1_arch [0:ISSUE_WIDTH-1], // architectural rs1 per slot
    input  logic [4:0]              rs2_arch [0:ISSUE_WIDTH-1], // architectural rs2 per slot
    output logic [PHYS_TAG_W-1:0]   rs1_phys [0:ISSUE_WIDTH-1], // physical tag for rs1
    output logic [PHYS_TAG_W-1:0]   rs2_phys [0:ISSUE_WIDTH-1], // physical tag for rs2

    // write interface - destination register rename
    input  logic [ISSUE_WIDTH-1:0]  wr_en,                      // write enable per slot
    input  logic [4:0]              rd_arch  [0:ISSUE_WIDTH-1], // architectural rd per slot
    input  logic [PHYS_TAG_W-1:0]   rd_phys  [0:ISSUE_WIDTH-1], // new physical tag per slot

    // flush interface - restore from retire RAT on misprediction
    input  logic                    flush,
    input  logic [PHYS_TAG_W-1:0]   retire_rat_in [0:N_ARCH_REGS-1], // committed mapping

    // stale physical register output - needed by ROB to reclaim at commit
    output logic [PHYS_TAG_W-1:0]   stale_phys [0:ISSUE_WIDTH-1] // old mapping before rename
);

    // local parameters
    localparam ARCH_W = $clog2(N_ARCH_REGS); // bit width to index architectural register file

    // RAT storage
    logic [PHYS_TAG_W-1:0]  map [0:N_ARCH_REGS-1]; // mapping table: arch reg -> phys tag

    // bypass signals - intra-dispatch forwarding from slot 0 write to slot 1 read
    logic                   bypass_rs1_s1; // slot 1 rs1 matches slot 0 rd
    logic                   bypass_rs2_s1; // slot 1 rs2 matches slot 0 rd
    
    // 1. Sequential block
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
            if(flush)
            begin
                for(int i = 0; i < N_ARCH_REGS; i++)
                begin
                    map[i] <= retire_rat_in[i];
                end
            end
            else
            begin
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(wr_en[i] && rd_arch[i] != '0)
                    map[rd_arch[i]] <= rd_phys[i];
                end
            end
        end
    end
    
    // 2. Combinational block
    always_comb
    begin
        // For Bypass
        bypass_rs1_s1 = (wr_en[0] && (rd_arch[0] != '0) && (rd_arch[0] == rs1_arch[1]));
        bypass_rs2_s1 = (wr_en[0] && (rd_arch[0] != '0) && (rd_arch[0] == rs2_arch[1]));
        
        rs1_phys[0] = map[rs1_arch[0]];
        rs2_phys[0] = map[rs2_arch[0]];
        rs1_phys[1] = bypass_rs1_s1 ? rd_phys[0] : map[rs1_arch[1]];
        rs2_phys[1] = bypass_rs2_s1 ? rd_phys[0] : map[rs2_arch[1]];
        
         // Stale Physical Register Mapping
         stale_phys[0] = wr_en[0] ? map[rd_arch[0]] : '0;
         if(wr_en[1])
         begin
            if((wr_en[0] && (rd_arch[0] != '0) && (rd_arch[0] == rd_arch[1])))
            begin
                stale_phys[1] = rd_phys[0];
            end
            else
                stale_phys[1] = map[rd_arch[1]];
         end
         else
            stale_phys[1] = '0;         
    end    
endmodule