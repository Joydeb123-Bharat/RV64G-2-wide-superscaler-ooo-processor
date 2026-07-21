`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 22:06:04
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: prf
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Physical Register File (PRF) with N_PHYS_REGS=96 entries, each
//              DATA_W=64 bits wide. Provides N_EXEC_UNITS*2=8 combinational read
//              ports (2 sources per execution unit), N_EXEC_UNITS=4 write ports
//              (one per execution unit result bus), ISSUE_WIDTH=2 invalidate ports
//              (clear valid bit when a new physical tag is allocated at rename),
//              and ISSUE_WIDTH*2=4 dispatch-valid query ports (used to initialise
//              IQ src_rdy bits at dispatch). On flush, all valid bits except the
//              initial identity-mapped 32 architectural registers are cleared.
//
// Dependencies: (none — register array)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Physical tags 0–31 are identity-mapped to architectural registers x0–x31 at
//   reset and are always valid. Tags 32–95 form the free pool (N_PHYS_REGS - N_ARCH_REGS = 64).
//
//////////////////////////////////////////////////////////////////////////////////


module prf #(
    parameter N_PHYS_REGS   = 96,   // number of physical register entries
    parameter DATA_W        = 64,   // data width of each entry
    parameter PHYS_TAG_W    = 7,    // ceil(log2(N_PHYS_REGS))
    parameter N_EXEC_UNITS  = 4,    // number of execution units
    parameter N_READ_PORTS  = 8,    // 2 sources per execution unit x N_EXEC_UNITS
    parameter N_WRITE_PORTS = 4,    // one write port per execution unit
    parameter ISSUE_WIDTH   = 2     // instructions dispatched per cycle
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // read interface - combinational reads from execution units
    input  logic [PHYS_TAG_W-1:0]     rd_tag    [0:N_READ_PORTS-1],   // physical tag to read
    output logic [DATA_W-1:0]         rd_data   [0:N_READ_PORTS-1],   // data read from PRF
    output logic                      rd_valid  [0:N_READ_PORTS-1],   // valid bit for read tag

    // write interface - result written by execution units at complete
    input  logic [N_WRITE_PORTS-1:0]  wr_en,                          // write enable per port
    input  logic [PHYS_TAG_W-1:0]     wr_tag    [0:N_WRITE_PORTS-1],  // physical tag to write
    input  logic [DATA_W-1:0]         wr_data   [0:N_WRITE_PORTS-1],  // data to write

    // invalidate interface - clear valid bit when tag allocated at rename
    input  logic [ISSUE_WIDTH-1:0]    inv_en,                          // invalidate enable per slot
    input  logic [PHYS_TAG_W-1:0]     inv_tag   [0:ISSUE_WIDTH-1],    // tag to invalidate

    // dispatch valid check - combinational valid bit read for IQ src_rdy initialisation
    input  logic [PHYS_TAG_W-1:0]     disp_tag  [0:ISSUE_WIDTH*2-1], // src tags at dispatch
    output logic                      disp_valid [0:ISSUE_WIDTH*2-1], // valid bits for src tags

    // flush interface
    input  logic                      flush                            // global flush signal
);

    // local parameters
    localparam ARCH_REGS = 32;  // initial identity mapped registers always valid at reset

    // PRF storage
    logic [DATA_W-1:0]   prf_data  [0:N_PHYS_REGS-1];  // data storage per physical register
    logic                prf_valid [0:N_PHYS_REGS-1];  // valid bit per physical register
    
    // 1. Seqquential Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            for(int i = 0; i < N_PHYS_REGS; i++)
            begin
                prf_data[i] <= '0;
                prf_valid[i] <= (i < ARCH_REGS) ? 1'b1 : 1'b0;
            end
        end
        else
        begin
            if(!flush)
            begin
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin: INVALID
                    if(inv_en[i])
                        prf_valid[inv_tag[i]] <= 1'b0;
                end
            end

            // Writes are NOT gated by flush: JAL/JALR return-address writes fire
            // in the same cycle as the flush signal, and must still commit to PRF.
            // Wrong-path writes during flush are harmless because RAT rollback
            // ensures those physical registers are never referenced again.
            for(int i = 0; i < N_WRITE_PORTS; i++)
            begin: WRITE
                if(wr_en[i] && (wr_tag[i] != '0))
                begin
                    prf_data[wr_tag[i]] <= wr_data[i];
                    prf_valid[wr_tag[i]] <= 1'b1;
                end
            end
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        for(int i = 0; i < N_READ_PORTS; i++)
        begin
            rd_data[i] = prf_data[rd_tag[i]];
            rd_valid[i]= prf_valid[rd_tag[i]];
            for(int b = 0; b < N_WRITE_PORTS; b++)
            begin : FORWARDING
                if(wr_en[b] && (wr_tag[b] != '0) && (wr_tag[b] == rd_tag[i]))
                begin
                    rd_valid[i] = 1'b1;
                    rd_data[i] = wr_data[b];
                end
            end
            
        end
        for(int i = 0; i < 2*ISSUE_WIDTH; i++)
        begin
            disp_valid[i] = prf_valid[disp_tag[i]];
            for(int b = 0; b < N_WRITE_PORTS; b++)
            begin : DISP_FORWARDING
                if(wr_en[b] && (wr_tag[b] != '0) && (wr_tag[b] == disp_tag[i]))
                    disp_valid[i] = 1'b1;
            end
        end
    end
    
endmodule