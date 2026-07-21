`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 10:27:27
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: alu_64
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: 64-bit integer Arithmetic Logic Unit (ALU). Supports all RV64I
//              integer operations: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT,
//              SLTU, LUI (pass immediate), AUIPC (PC + immediate). The is_word
//              flag enables 32-bit mode for W-suffix instructions (ADDW, SUBW,
//              SLLW, etc.): operands are sign-extended from their lower 32 bits
//              before the operation, and the result is sign-extended back to 64
//              bits. Purely combinational — no registers, no clock.
//
// Dependencies: (none — combinational datapath)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   alu_op encoding: 4'b0000=ADD 4'b0001=SLL 4'b0010=SLT 4'b0011=SLTU
//   4'b0100=XOR 4'b0101=SRL 4'b0110=OR  4'b0111=AND 4'b1000=SUB
//   4'b1101=SRA  4'b1110=LUI  4'b1111=AUIPC
//   Two instances used in ooo_core: u_alu0 (unit slot 0) and u_alu1 (unit slot 1).
//
//////////////////////////////////////////////////////////////////////////////////

module alu_64 #(
    parameter DATA_W = 64  // operand and result width
)(
    // operand inputs
    input  logic [DATA_W-1:0]  src1,       // first source operand read from PRF
    input  logic [DATA_W-1:0]  src2,       // second source operand or sign-extended immediate
    input  logic [3:0]         alu_op,     // operation select code
    input  logic               is_word,    // when high operate on lower 32 bits and sign-extend result

    // result output
    output logic [DATA_W-1:0]  result      // computed result written to PRF
);

    // local parameters - operation encoding
    localparam ALU_ADD   = 4'b0000;  // addition src1 + src2 funct3=000 funct7=normal
    localparam ALU_SLL   = 4'b0001;  // logical left shift src1 << src2[5:0] funct3=001
    localparam ALU_SLT   = 4'b0010;  // set less than signed result=1 if src1<src2 funct3=010
    localparam ALU_SLTU  = 4'b0011;  // set less than unsigned funct3=011
    localparam ALU_XOR   = 4'b0100;  // bitwise XOR funct3=100
    localparam ALU_SRL   = 4'b0101;  // logical right shift fills with zero funct3=101 funct7=normal
    localparam ALU_OR    = 4'b0110;  // bitwise OR funct3=110
    localparam ALU_AND   = 4'b0111;  // bitwise AND funct3=111
    localparam ALU_SUB   = 4'b1000;  // subtraction src1 - src2 funct3=000 funct7=alt
    localparam ALU_SRA   = 4'b1101;  // arithmetic right shift fills with sign bit funct3=101 funct7=alt
    localparam ALU_LUI   = 4'b1110;  // load upper immediate passes src2 directly to result
    localparam ALU_AUIPC = 4'b1111;  // add upper immediate to PC src1=PC src2=imm


    // internal signals
    logic [DATA_W-1:0]   op_a;      // operand A after word truncation if is_word
    logic [DATA_W-1:0]   op_b;      // operand B after word truncation if is_word
    logic [DATA_W-1:0]   raw_result; // result before sign extension
    logic [4:0]          shamt_w;   // 5-bit shift amount for word shifts
    logic [5:0]          shamt_d;   // 6-bit shift amount for doubleword shifts
    
    always_comb
    begin
        op_a = is_word ? { {32{src1[31]}} , src1[31:0]} : src1;
        op_b = is_word ? { {32{src2[31]}} , src2[31:0]} : src2;
        shamt_w = src2[4:0];
        shamt_d = src2[5:0];
        
        case(alu_op)
            ALU_ADD  : raw_result = op_a + op_b;
            ALU_SUB  : raw_result = op_a - op_b;
            ALU_AND  : raw_result = op_a & op_b;
            ALU_OR   : raw_result = op_a | op_b;
            ALU_XOR  : raw_result = op_a ^ op_b;
            ALU_SLL  : raw_result = op_a << (is_word ? shamt_w : shamt_d);
            ALU_SRL  : raw_result = op_a >> (is_word ? shamt_w : shamt_d);
            ALU_SRA  : raw_result = $signed(op_a) >>> (is_word ? shamt_w : shamt_d);
            ALU_SLT  : raw_result = ($signed(op_a) < $signed(op_b)) ? 64'b1 : 64'b0;
            ALU_SLTU : raw_result = (op_a < op_b) ? 64'b1 : 64'b0;
            ALU_LUI  : raw_result = src2;
            ALU_AUIPC: raw_result = src1 + src2;
            default  : raw_result = '0;
        endcase
        result = is_word ? { {32{raw_result[31]}} , raw_result[31:0] } : raw_result;
    end
endmodule