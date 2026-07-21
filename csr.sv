`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 17.07.2026 16:25:09
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: csr
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Machine-mode Control and Status Registers (CSR) block per the
//              RISC-V Privileged Architecture specification. Implements:
//              mtvec  — trap vector base address (reset: 0x0000_0000_0000_1000),
//              mepc   — exception program counter (written on exception commit),
//              mcause — exception cause code (written on exception commit),
//              mtval  — trap value / faulting address (written on exception commit),
//              mstatus[3]=MIE, mstatus[7]=MPIE — interrupt enable bits,
//              minstret — instruction retire counter (incremented by commit_count),
//              mcycle  — free-running cycle counter.
//              Provides mtvec_out to ROB for exception redirect target and
//              mepc_out to fetch unit via MRET path.
//
// Dependencies: (none — register array)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   MRET support is stubbed (mret_en input wired to 1'b0 in ooo_core).
//   Only machine-mode CSRs are implemented; S/U mode and PMP are not included.
//
//////////////////////////////////////////////////////////////////////////////////


module csr #(
    parameter DATA_W = 64   // register width matches XLEN
)(
    input  logic              clk,
    input  logic              rst_n,

    // flush interface
    input  logic              flush,           // suppress writes from squashed instructions

    // exception commit interface - written by ROB when exception reaches HEAD
    input  logic              exc_commit_en,   // exception being committed this cycle
    input  logic [DATA_W-1:0] exc_pc_in,       // faulting PC written to mepc
    input  logic [3:0]        exc_cause_in,    // cause code written to mcause lower bits
    input  logic [DATA_W-1:0] exc_val_in,      // faulting address or bad instruction for mtval

    // MRET interface - machine return from trap
    input  logic              mret_en,         // MRET instruction executing this cycle
    output logic [DATA_W-1:0] mret_pc_out,     // mepc value sent to fetch unit on MRET

    // trap handler address - read by ROB for flush_target on exception
    output logic [DATA_W-1:0] mtvec_out,       // trap vector base address

    // exception info outputs
    output logic [DATA_W-1:0] mepc_out,        // last exception PC
    output logic [DATA_W-1:0] mcause_out,      // last exception cause

    // performance counter interface
    input  logic [1:0]        commit_count,    // instructions retiring this cycle for minstret

    // mstatus bit outputs
    output logic              mie,             // machine interrupt enable bit from mstatus[3]
    output logic              mpie             // machine previous interrupt enable from mstatus[7]
);

    // local parameters - CSR addresses per RISC-V privileged specification
    localparam MTVEC_RESET = 64'h0000_0000_0000_1000; // default trap handler address at reset

    // CSR storage registers
    logic [DATA_W-1:0]  mstatus;   // machine status register bit3=mie bit7=mpie
    logic [DATA_W-1:0]  mtvec;     // trap vector base address written by firmware at boot
    logic [DATA_W-1:0]  mepc;      // PC of faulting instruction written on exception commit
    logic [DATA_W-1:0]  mcause;    // exception cause code written on exception commit
    logic [DATA_W-1:0]  mtval;     // faulting address or bad instruction written on exception
    logic [DATA_W-1:0]  mcycle;    // free-running cycle counter increments every clock
    logic [DATA_W-1:0]  minstret;  // retired instruction counter increments at commit
        
    // 1. Sequentail Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            mstatus  <= '0; 
            mtvec    <= MTVEC_RESET;   
            mepc     <= '0;    
            mcause   <= '0;  
            mtval    <= '0;   
            mcycle   <= '0;  
            minstret <= '0;
        end
        else
        begin
            if(exc_commit_en)
            begin
                mepc <= {exc_pc_in[DATA_W-1:2],2'b0};
                mcause<= {60'b0, exc_cause_in};
                mtval <= exc_val_in;
                mstatus[7] <= mstatus[3];  // mpie <= mie
                mstatus[3] <= 1'b0;        // mie <= 0 disable interrupts during trap
            end
            if(mret_en && !flush)
            begin
                mstatus[3] <= mstatus[7];  // mie <= mpie
                mstatus[7] <= 1'b1;        // mpie <= 1
            end
            mcycle <= mcycle + 1'd1;
            minstret <= minstret + commit_count;
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        mie = mstatus[3];
        mpie= mstatus[7];
        
        mret_pc_out = '0;
        mtvec_out = mtvec;
        mepc_out = mepc;
        mcause_out = mcause;
        
        if(mret_en)
            mret_pc_out = mepc;
    end
    
endmodule