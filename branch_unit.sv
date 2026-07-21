`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 11:24:48
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: branch_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Combinational branch resolution unit. Evaluates BEQ, BNE, BLT,
//              BGE, BLTU, BGEU conditions using src1/src2 operands from the PRF.
//              Computes the actual branch target (PC + imm for conditional branches,
//              PC + imm for JAL, src1 + imm for JALR with bit 0 cleared per spec).
//              Detects misprediction by comparing actual_taken vs. predicted_taken
//              and actual_target vs. predicted_target. For JAL and JALR, writes
//              the return address (instr_pc + 4) to the PRF via rd_wr_en/rd_phys/
//              rd_value. Raises misaligned-target exception when actual_target[1:0] != 0.
//
// Dependencies: (none — combinational)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   Shares ALU0 issue slot (issue_valid[0]) — activated when is_branch | is_jal |
//   is_jalr is set in that slot. actual_taken drives the branch predictor update.
//
//////////////////////////////////////////////////////////////////////////////////

module branch_unit #(
    parameter DATA_W     = 64,  // operand and PC width
    parameter ROB_IDX_W  = 6,   // bit width of ROB index
    parameter PHYS_TAG_W = 7    // bit width of physical register tag
)(
    // unit enable
    input  logic                  valid,             // high when unit has valid instruction

    // operand inputs from PRF
    input  logic [DATA_W-1:0]     src1,              // first source operand
    input  logic [DATA_W-1:0]     src2,              // second source operand
    input  logic [DATA_W-1:0]     imm,               // sign-extended immediate
    input  logic [DATA_W-1:0]     instr_pc,          // PC of this branch instruction
    input  logic [2:0]            funct3,            // branch type encoding
    input  logic                  is_jalr,           // high for JALR instruction
    input  logic                  is_jal,            // high for JAL instruction

    // prediction inputs from IQ entry
    input  logic                  predicted_taken,   // predictor guess stored at dispatch
    input  logic [DATA_W-1:0]     predicted_target,  // predictor target stored at dispatch

    // ROB writeback tag
    input  logic [ROB_IDX_W-1:0]  rob_idx,           // ROB slot to write completion to
    input  logic [PHYS_TAG_W-1:0] phys_dst,          // physical destination for JAL/JALR

    // outputs to ROB execute complete interface
    output logic                  complete,          // asserted when valid result produced
    output logic                  mispredicted,      // predictor was wrong
    output logic                  exception,         // misaligned target address
    output logic [3:0]            exc_cause,         // exception cause code
    output logic [DATA_W-1:0]     actual_target,     // correct fetch PC after resolution
    output logic [ROB_IDX_W-1:0]  result_rob_idx,    // echoed ROB index for writeback

    // outputs to PRF for JAL/JALR return address
    output logic                  rd_wr_en,          // high when return address written to PRF
    output logic [PHYS_TAG_W-1:0] rd_phys,           // physical destination tag
    output logic [DATA_W-1:0]     rd_value,          // return address PC+4

    // resolved outputs
    output logic                  actual_taken       // resolved taken or not taken
);

    // local parameters - funct3 branch type encoding
    localparam BEQ  = 3'b000;  // branch if equal
    localparam BNE  = 3'b001;  // branch if not equal
    localparam BLT  = 3'b100;  // branch if less than signed
    localparam BGE  = 3'b101;  // branch if greater or equal signed
    localparam BLTU = 3'b110;  // branch if less than unsigned
    localparam BGEU = 3'b111;  // branch if greater or equal unsigned

    // internal signals
    logic [DATA_W-1:0]  branch_target;  // instr_pc + imm for taken conditional or JAL
    logic [DATA_W-1:0]  jalr_target;    // src1 + imm with LSB cleared for JALR
    logic [DATA_W-1:0]  fallthrough;    // instr_pc + 4 for not-taken path
    
    always_comb
    begin
        complete    = valid ? 1'b1 : 1'b0;
        fallthrough = instr_pc + 64'd4;
        rd_wr_en    = valid ? (is_jalr | is_jal) : 1'b0;
        rd_value    = rd_wr_en ? fallthrough : '0;
        rd_phys     = rd_wr_en ? phys_dst : '0;
        jalr_target = valid ? (src1 + imm) & (~64'b1) : '0;
        
        if(is_jalr || is_jal)
        begin
            actual_taken = 1'b1;
        end
        else if(valid)
        begin
            case(funct3)
                BEQ  : actual_taken = (src1 == src2);
                BNE  : actual_taken = (src1 != src2);
                BLT  : actual_taken = ($signed(src1) < $signed(src2));
                BGE  : actual_taken = ($signed(src1) >= $signed(src2));
                BLTU : actual_taken = (src1 < src2);
                BGEU : actual_taken = (src1 >= src2);
                default: actual_taken = '0;
            endcase
        end
        else
            actual_taken = '0;
            
        branch_target = instr_pc + imm;    
        actual_target = is_jalr ? jalr_target : ((is_jal | actual_taken) ? branch_target : fallthrough) ;
        result_rob_idx = rob_idx;
        mispredicted = valid ? (actual_taken != predicted_taken) | (actual_taken && (predicted_target != actual_target)) : '0;
        
        exc_cause = '0;
        exception = valid & actual_taken & (actual_target[1:0] != '0);
    end
    
endmodule