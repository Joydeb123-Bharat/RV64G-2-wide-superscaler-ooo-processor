`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 18.07.2026 19:30:43
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: ld_st_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Address Generation Unit (AGU) for load and store instructions.
//              Combinationally computes the effective address as src1 (base register
//              from PRF) + imm (sign-extended byte offset from IQ). Routes the result
//              to the LDQ or STQ via the LSQ execute interface (ld_exec / st_exec
//              buses). Checks natural alignment and raises a misaligned-address
//              exception (exception=1, exc_cause set) to the ROB if violated.
//              mispredicted_out and branch_target_out are always 0 (not a branch unit).
//
// Dependencies: lsq (target of ld_exec/st_exec outputs)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Alignment check: byte=none, halfword=addr[0]==0, word=addr[1:0]==00,
//   doubleword=addr[2:0]==000. Issues complete to ROB in the same cycle.
//
//////////////////////////////////////////////////////////////////////////////////


module ld_st_unit #(
    parameter DATA_W     = 64,  // operand and address width
    parameter ROB_IDX_W  = 6,   // bit width of ROB index
    parameter PHYS_TAG_W = 7    // bit width of physical register tag
)(
    // input from issue queue
    input  logic                   valid_in,        // unit has valid instruction this cycle
    input  logic [DATA_W-1:0]      src1_in,         // base register value from PRF
    input  logic [DATA_W-1:0]      src2_in,         // store data value from PRF
    input  logic [DATA_W-1:0]      imm_in,          // sign-extended byte offset
    input  logic                   is_load_in,      // instruction is a load
    input  logic                   is_store_in,     // instruction is a store
    input  logic [1:0]             mem_size_in,     // 00=byte 01=half 10=word 11=double
    input  logic [ROB_IDX_W-1:0]   rob_idx_in,      // ROB slot for execute complete
    input  logic [4:0]             lsq_idx_in,      // LSQ slot allocated at dispatch

    // outputs to LSQ execute interface
    output logic                   ld_exec_valid,   // load address computed this cycle
    output logic [4:0]             ld_exec_idx,     // which LDQ entry to update
    output logic [DATA_W-1:0]      ld_exec_addr,    // computed load effective address
    output logic                   st_exec_valid,   // store address and data ready
    output logic [4:0]             st_exec_idx,     // which STQ entry to update
    output logic [DATA_W-1:0]      st_exec_addr,    // computed store effective address
    output logic [DATA_W-1:0]      st_exec_data,    // store data to write to STQ

    // outputs to ROB execute complete interface
    output logic                   complete,        // address computation done
    output logic [ROB_IDX_W-1:0]   rob_idx_out,     // echoed ROB index for writeback
    output logic                   exception,       // misaligned address detected
    output logic [3:0]             exc_cause,       // exception cause code
    output logic                   mispredicted_out,// always 0 loads stores never mispredict
    output logic [DATA_W-1:0]      branch_target_out // always 0 not a branch unit
);

    // local parameters - memory access alignment requirements
    localparam MEM_BYTE   = 2'b00;  // byte access no alignment requirement
    localparam MEM_HALF   = 2'b01;  // halfword must be 2-byte aligned addr[0]=0
    localparam MEM_WORD   = 2'b10;  // word must be 4-byte aligned addr[1:0]=00
    localparam MEM_DOUBLE = 2'b11;  // doubleword must be 8-byte aligned addr[2:0]=000

    // exception cause codes per RISC-V spec
    localparam EXC_LOAD_MISALIGN  = 4'd4;  // load address misaligned
    localparam EXC_STORE_MISALIGN = 4'd6;  // store address misaligned

    // internal signals
    logic [DATA_W-1:0]  eff_addr;      // computed effective address src1 + imm
    logic               misaligned;    // address violates alignment requirement
    
    // Combinational BLock
    always_comb
    begin
        eff_addr         = '0;
        misaligned       = 0;
        ld_exec_valid    = 0;
        ld_exec_idx      = '0;
        ld_exec_addr     = '0;
        st_exec_valid    = 0;
        st_exec_idx      = '0;
        st_exec_addr     = '0;
        st_exec_data     = '0;
        complete         = 0;
        rob_idx_out      = '0;
        exception        = 0;
        exc_cause        = '0;
        mispredicted_out = 0;
        branch_target_out= '0;
        
        if(valid_in)
        begin
            eff_addr = src1_in + imm_in;
            rob_idx_out = rob_idx_in;
            complete = 1'b1;
            
            case(mem_size_in)
                MEM_BYTE  : misaligned = 1'b0;
                MEM_HALF  : misaligned = eff_addr[0];
                MEM_WORD  : misaligned = |eff_addr[1:0];
                MEM_DOUBLE: misaligned =  |eff_addr[2:0];
            endcase
            
            if(misaligned)
            begin
                if(is_load_in)
                begin
                    exception = 1'b1;
                    exc_cause= EXC_LOAD_MISALIGN; 
                end
                if(is_store_in)
                begin
                    exception = 1'b1;
                    exc_cause= EXC_STORE_MISALIGN; 
                end
            end
            else
            begin
                if(is_load_in)
                begin
                    ld_exec_valid = 1'b1;
                    ld_exec_idx   = lsq_idx_in;
                    ld_exec_addr  = eff_addr;
                end
                if(is_store_in)
                begin
                    st_exec_valid = 1'b1;
                    st_exec_idx   = lsq_idx_in;
                    st_exec_addr  = eff_addr;
                    st_exec_data  = src2_in;
                end
            end
        end
    end
    
endmodule