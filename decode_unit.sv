`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 18.07.2026 11:53:53
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: decode_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Registered instruction decode stage. Decodes up to ISSUE_WIDTH (2)
//              RV64G (RV64I + M-extension) instructions per cycle. Extracts register
//              fields (rs1, rs2, rd), reconstructs all immediate formats (I/S/B/U/J),
//              generates ALU operation codes, unit-select (ALU/MULDIV/LDST), memory
//              access size, and type flags (load, store, branch, JAL, JALR, MUL,
//              DIV, word, illegal). Stalls on back-pressure and clears valid bits
//              on flush.
//
// Dependencies: (none)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   ALU op codes (dec_alu_op): 0=ADD 1=SLL 2=SLT 3=SLTU 4=XOR 5=SRL 6=OR 7=AND
//   8=SUB 9=SRA(funct7[5]) 10=LUI 11=AUIPC — must match alu_64.sv localparams.
//   Unit select: 2'b00=ALU  2'b01=MULDIV  2'b10=LDST  2'b11=reserved.
//
//////////////////////////////////////////////////////////////////////////////////


module decode_unit #(
    parameter DATA_W      = 64,
    parameter ISSUE_WIDTH = 2
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // input from fetch unit
    input  logic [ISSUE_WIDTH-1:0]    fetch_valid,
    input  logic [31:0]               fetch_instr    [0:ISSUE_WIDTH-1],
    input  logic [DATA_W-1:0]         fetch_pc       [0:ISSUE_WIDTH-1],

    // stall and flush
    input  logic                      stall,
    input  logic                      flush,

    // decoded outputs to dispatch unit
    output logic [ISSUE_WIDTH-1:0]    dec_valid,
    output logic [4:0]                dec_rs1        [0:ISSUE_WIDTH-1],
    output logic [4:0]                dec_rs2        [0:ISSUE_WIDTH-1],
    output logic [4:0]                dec_rd         [0:ISSUE_WIDTH-1],
    output logic [DATA_W-1:0]         dec_imm        [0:ISSUE_WIDTH-1],
    output logic [DATA_W-1:0]         dec_pc         [0:ISSUE_WIDTH-1],
    output logic [3:0]                dec_alu_op     [0:ISSUE_WIDTH-1],
    output logic [2:0]                dec_funct3     [0:ISSUE_WIDTH-1],
    output logic [1:0]                dec_unit_sel   [0:ISSUE_WIDTH-1],
    output logic [1:0]                dec_mem_size   [0:ISSUE_WIDTH-1],
    output logic [ISSUE_WIDTH-1:0]    dec_uses_imm,
    output logic [ISSUE_WIDTH-1:0]    dec_is_load,
    output logic [ISSUE_WIDTH-1:0]    dec_is_store,
    output logic [ISSUE_WIDTH-1:0]    dec_is_branch,
    output logic [ISSUE_WIDTH-1:0]    dec_is_jalr,
    output logic [ISSUE_WIDTH-1:0]    dec_is_jal,
    output logic [ISSUE_WIDTH-1:0]    dec_is_mul,
    output logic [ISSUE_WIDTH-1:0]    dec_is_div,
    output logic [ISSUE_WIDTH-1:0]    dec_sign_ext,
    output logic [ISSUE_WIDTH-1:0]    dec_has_rd,
    output logic [ISSUE_WIDTH-1:0]    dec_is_word,
    output logic [ISSUE_WIDTH-1:0]    dec_illegal
);

    // RISC-V opcodes
    localparam OPC_LOAD   = 7'b0000011;
    localparam OPC_STORE  = 7'b0100011;
    localparam OPC_IMM    = 7'b0010011;
    localparam OPC_REG    = 7'b0110011;
    localparam OPC_LUI    = 7'b0110111;
    localparam OPC_AUIPC  = 7'b0010111;
    localparam OPC_JAL    = 7'b1101111;
    localparam OPC_JALR   = 7'b1100111;
    localparam OPC_BRANCH = 7'b1100011;
    localparam OPC_IMM32  = 7'b0011011;
    localparam OPC_REG32  = 7'b0111011;

    // funct7 encodings
    localparam F7_NORMAL = 7'b0000000;  // standard integer operations
    localparam F7_ALT    = 7'b0100000;  // SUB SRA SUBW SRAW
    localparam F7_MEXT   = 7'b0000001;  // M-extension MUL DIV REM

    // ALU funct3 encodings
    localparam F3_ADD_SUB = 3'b000;  // ADD SUB ADDI
    localparam F3_SLL     = 3'b001;  // SLL SLLI
    localparam F3_SLT     = 3'b010;  // SLT SLTI
    localparam F3_SLTU    = 3'b011;  // SLTU SLTIU
    localparam F3_XOR     = 3'b100;  // XOR XORI
    localparam F3_SRL_SRA = 3'b101;  // SRL SRA SRLI SRAI
    localparam F3_OR      = 3'b110;  // OR ORI
    localparam F3_AND     = 3'b111;  // AND ANDI

    // branch funct3 encodings
    localparam F3_BEQ  = 3'b000;  // branch if equal
    localparam F3_BNE  = 3'b001;  // branch if not equal
    localparam F3_BLT  = 3'b100;  // branch if less than signed
    localparam F3_BGE  = 3'b101;  // branch if greater or equal signed
    localparam F3_BLTU = 3'b110;  // branch if less than unsigned
    localparam F3_BGEU = 3'b111;  // branch if greater or equal unsigned

    // load funct3 encodings
    localparam F3_LB  = 3'b000;  // load byte signed
    localparam F3_LH  = 3'b001;  // load halfword signed
    localparam F3_LW  = 3'b010;  // load word signed
    localparam F3_LD  = 3'b011;  // load doubleword
    localparam F3_LBU = 3'b100;  // load byte unsigned
    localparam F3_LHU = 3'b101;  // load halfword unsigned
    localparam F3_LWU = 3'b110;  // load word unsigned

    // store funct3 encodings
    localparam F3_SB  = 3'b000;  // store byte
    localparam F3_SH  = 3'b001;  // store halfword
    localparam F3_SW  = 3'b010;  // store word
    localparam F3_SD  = 3'b011;  // store doubleword

    // ALU operation codes - must match alu_64.sv
    localparam ALU_ADD   = 4'd0;
    localparam ALU_SLL   = 4'd1;
    localparam ALU_SLT   = 4'd2;
    localparam ALU_SLTU  = 4'd3;
    localparam ALU_XOR   = 4'd4;
    localparam ALU_SRL   = 4'd5;
    localparam ALU_OR    = 4'd6;
    localparam ALU_AND   = 4'd7;
    localparam ALU_SUB   = 4'd8;
    localparam ALU_SRA   = 4'd13;
    localparam ALU_LUI   = 4'd14;
    localparam ALU_AUIPC = 4'd15;

    // MULDIV operation codes - lower 3 bits match funct3 directly
    localparam MUL_MUL    = 4'd0;  // funct3=000 lower 64 bits
    localparam MUL_MULH   = 4'd1;  // funct3=001 upper signed x signed
    localparam MUL_MULHSU = 4'd2;  // funct3=010 upper signed x unsigned
    localparam MUL_MULHU  = 4'd3;  // funct3=011 upper unsigned x unsigned
    localparam DIV_DIV    = 4'd4;  // funct3=100 signed quotient
    localparam DIV_DIVU   = 4'd5;  // funct3=101 unsigned quotient
    localparam DIV_REM    = 4'd6;  // funct3=110 signed remainder
    localparam DIV_REMU   = 4'd7;  // funct3=111 unsigned remainder

    // unit select encoding - must match issue_queue.sv
    localparam UNIT_ALU    = 2'b00;  // ALU0 or ALU1
    localparam UNIT_MULDIV = 2'b01;  // MUL DIV unit
    localparam UNIT_LDST   = 2'b10;  // load store unit
    localparam UNIT_RSVD   = 2'b11;  // reserved

    // memory access size encoding - must match lsq.sv
    localparam MEM_BYTE   = 2'b00;  // 1 byte
    localparam MEM_HALF   = 2'b01;  // 2 bytes
    localparam MEM_WORD   = 2'b10;  // 4 bytes
    localparam MEM_DOUBLE = 2'b11;  // 8 bytes

    // raw field extractions per slot
    logic [6:0]  opcode  [0:ISSUE_WIDTH-1];  // bits 6:0
    logic [4:0]  rs1_raw [0:ISSUE_WIDTH-1];  // bits 19:15
    logic [4:0]  rs2_raw [0:ISSUE_WIDTH-1];  // bits 24:20
    logic [4:0]  rd_raw  [0:ISSUE_WIDTH-1];  // bits 11:7
    logic [2:0]  funct3  [0:ISSUE_WIDTH-1];  // bits 14:12
    logic [6:0]  funct7  [0:ISSUE_WIDTH-1];  // bits 31:25

    // immediate reconstructions per slot
    logic [DATA_W-1:0] imm_i [0:ISSUE_WIDTH-1];  // I-type sign extended
    logic [DATA_W-1:0] imm_s [0:ISSUE_WIDTH-1];  // S-type sign extended
    logic [DATA_W-1:0] imm_b [0:ISSUE_WIDTH-1];  // B-type sign extended
    logic [DATA_W-1:0] imm_u [0:ISSUE_WIDTH-1];  // U-type sign extended
    logic [DATA_W-1:0] imm_j [0:ISSUE_WIDTH-1];  // J-type sign extended

    // combinational decode results per slot
    logic [ISSUE_WIDTH-1:0]    dec_valid_c;
    logic [4:0]                dec_rs1_c      [0:ISSUE_WIDTH-1];
    logic [4:0]                dec_rs2_c      [0:ISSUE_WIDTH-1];
    logic [4:0]                dec_rd_c       [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         dec_imm_c      [0:ISSUE_WIDTH-1];
    logic [DATA_W-1:0]         dec_pc_c       [0:ISSUE_WIDTH-1];
    logic [3:0]                dec_alu_op_c   [0:ISSUE_WIDTH-1];
    logic [2:0]                dec_funct3_c   [0:ISSUE_WIDTH-1];
    logic [1:0]                dec_unit_sel_c [0:ISSUE_WIDTH-1];
    logic [1:0]                dec_mem_size_c [0:ISSUE_WIDTH-1];
    logic [ISSUE_WIDTH-1:0]    dec_uses_imm_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_load_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_store_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_branch_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_jalr_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_jal_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_mul_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_div_c;
    logic [ISSUE_WIDTH-1:0]    dec_sign_ext_c;
    logic [ISSUE_WIDTH-1:0]    dec_has_rd_c;
    logic [ISSUE_WIDTH-1:0]    dec_is_word_c;
    logic [ISSUE_WIDTH-1:0]    dec_illegal_c;
    
    // 1. Sequentail Block
    always_ff @(posedge clk)
    begin
        if(!rst_n)
        begin
            dec_valid    <= '0;
            for(int i = 0; i < ISSUE_WIDTH; i++)
            begin
                dec_rs1[i]     <= '0;
                dec_rs2[i]     <= '0;
                dec_rd[i]      <= '0;
                dec_imm[i]     <= '0;
                dec_pc[i]      <= '0;
                dec_alu_op[i]  <= '0;
                dec_funct3[i]  <= '0;
                dec_unit_sel[i]<= '0;
                dec_mem_size[i]<= '0;
            end
            dec_uses_imm  <= '0;
            dec_is_load   <= '0;
            dec_is_store  <= '0;
            dec_is_branch <= '0;
            dec_is_jalr   <= '0;
            dec_is_jal    <= '0;
            dec_is_mul    <= '0;
            dec_is_div    <= '0;
            dec_sign_ext  <= '0;
            dec_has_rd    <= '0;
            dec_is_word   <= '0;
            dec_illegal   <= '0;
        end
        else
        begin
            if(flush)
            begin
                // flush clears valid bits only - wrong path instructions discarded
                dec_valid    <= '0;
                dec_is_load  <= '0;
                dec_is_store <= '0;
                dec_is_branch<= '0;
                dec_is_jalr  <= '0;
                dec_is_jal   <= '0;
                dec_is_mul   <= '0;
                dec_is_div   <= '0;
                dec_has_rd   <= '0;
                dec_illegal  <= '0;
            end
            else if(!stall)
            begin
                // normal operation - register all combinational decode outputs
                dec_valid    <= dec_valid_c;
                dec_uses_imm <= dec_uses_imm_c;
                dec_is_load  <= dec_is_load_c;
                dec_is_store <= dec_is_store_c;
                dec_is_branch<= dec_is_branch_c;
                dec_is_jalr  <= dec_is_jalr_c;
                dec_is_jal   <= dec_is_jal_c;
                dec_is_mul   <= dec_is_mul_c;
                dec_is_div   <= dec_is_div_c;
                dec_sign_ext <= dec_sign_ext_c;
                dec_has_rd   <= dec_has_rd_c;
                dec_is_word  <= dec_is_word_c;
                dec_illegal  <= dec_illegal_c;
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    dec_rs1[i]     <= dec_rs1_c[i];
                    dec_rs2[i]     <= dec_rs2_c[i];
                    dec_rd[i]      <= dec_rd_c[i];
                    dec_imm[i]     <= dec_imm_c[i];
                    dec_pc[i]      <= dec_pc_c[i];
                    dec_alu_op[i]  <= dec_alu_op_c[i];
                    dec_funct3[i]  <= dec_funct3_c[i];
                    dec_unit_sel[i]<= dec_unit_sel_c[i];
                    dec_mem_size[i]<= dec_mem_size_c[i];
                end
            end
            // stall - hold all outputs unchanged implicitly
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            // 1. Extract raw fields
            opcode[i]  = fetch_instr[i][6:0];
            rs1_raw[i] = fetch_instr[i][19:15];
            rs2_raw[i] = fetch_instr[i][24:20];
            rd_raw[i]  = fetch_instr[i][11:7];
            funct3[i]  = fetch_instr[i][14:12];
            funct7[i]  = fetch_instr[i][31:25];
        
            // 2. Reconstruct all immediates in parallel
            imm_i[i] = {{52{fetch_instr[i][31]}}, fetch_instr[i][31:20]};
            imm_s[i] = {{52{fetch_instr[i][31]}}, fetch_instr[i][31:25], fetch_instr[i][11:7]};
            imm_b[i] = {{51{fetch_instr[i][31]}}, fetch_instr[i][31], fetch_instr[i][7], 
                          fetch_instr[i][30:25], fetch_instr[i][11:8], 1'b0};
            imm_u[i] = {{32{fetch_instr[i][31]}}, fetch_instr[i][31:12], 12'b0};
            imm_j[i] = {{43{fetch_instr[i][31]}}, fetch_instr[i][31], fetch_instr[i][19:12], 
                          fetch_instr[i][20], fetch_instr[i][30:21], 1'b0};
        
            // 3. Set all defaults
            dec_valid_c[i]      = fetch_valid[i];
            dec_rs1_c[i]        = rs1_raw[i];
            dec_rs2_c[i]        = rs2_raw[i];
            dec_rd_c[i]         = rd_raw[i];
            dec_imm_c[i]        = imm_i[i];
            dec_pc_c[i]         = fetch_pc[i];
            dec_alu_op_c[i]     = ALU_ADD;
            dec_funct3_c[i]     = funct3[i];
            dec_unit_sel_c[i]   = UNIT_ALU;
            dec_mem_size_c[i]   = MEM_BYTE;
            dec_uses_imm_c[i]   = 1'b0;
            dec_is_load_c[i]    = 1'b0;
            dec_is_store_c[i]   = 1'b0;
            dec_is_branch_c[i]  = 1'b0;
            dec_is_jalr_c[i]    = 1'b0;
            dec_is_jal_c[i]     = 1'b0;
            dec_is_mul_c[i]     = 1'b0;
            dec_is_div_c[i]     = 1'b0;
            dec_sign_ext_c[i]   = 1'b0;
            dec_has_rd_c[i]     = 1'b0;
            dec_is_word_c[i]    = 1'b0;
            dec_illegal_c[i]    = 1'b0;
        
            // 4. Decode by opcode then funct3 then funct7
            if(fetch_valid[i] && !stall && !flush)
            begin
                case(opcode[i])
                    OPC_REG:   // R-type decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = rs2_raw[i];
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = '0;
                            dec_uses_imm_c[i]= '0;
                            dec_is_word_c[i] = '0;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);  
                            dec_funct3_c[i] = funct3[i];                         
                            case(funct7[i])
                                F7_NORMAL:
                                    begin
                                        dec_unit_sel_c[i] = UNIT_ALU;
                                        dec_alu_op_c[i]   = {1'b0, funct3[i]};
                                    end
                                F7_ALT:
                                    begin
                                        dec_unit_sel_c[i] = UNIT_ALU;
                                        case(funct3[i])
                                            3'b000: dec_alu_op_c[i] = ALU_SUB;
                                            3'b101: dec_alu_op_c[i] = ALU_SRA;
                                            default: dec_illegal_c[i] = 1'b1;
                                        endcase
                                    end
                                F7_MEXT:
                                    begin
                                        dec_unit_sel_c[i] = UNIT_MULDIV;
                                        dec_alu_op_c[i]   = {1'b0, funct3[i]};
                                        dec_is_mul_c[i]   = !funct3[i][2];       // funct3[2]=0 means MUL variant
                                        dec_is_div_c[i]   =  funct3[i][2];       // funct3[2]=1 means DIV variant
                                    end
                                 default: dec_illegal_c[i] = 1'b1;                                    
                             endcase
                            
                        end
                    OPC_IMM:   // I-type decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_i[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_is_word_c[i] = '0;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_ALU;
                            dec_alu_op_c[i]  = {1'b0, funct3[i]};
                            if(funct3[i] == F3_SRL_SRA)
                                dec_alu_op_c[i] = funct7[i][5] ? ALU_SRA : ALU_SRL;
                        end
                    OPC_LOAD:  // Load decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_i[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_is_load_c[i] = 1'b1;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_LDST;
                            dec_alu_op_c[i]  = ALU_ADD;
                            dec_mem_size_c[i]=funct3[i][1:0];
                            dec_sign_ext_c[i]=!funct3[i][2]; // funct3[2]=1 means unsigned; sign_ext=1 means sign-extend
                        end
                    OPC_STORE: // Store decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = rs2_raw[i];
                            dec_rd_c[i]  = '0;
                            dec_imm_c[i] = imm_s[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_is_store_c[i]= 1'b1;
                            dec_has_rd_c[i]  = 1'b0;    
                            dec_unit_sel_c[i]= UNIT_LDST;
                            dec_alu_op_c[i]  = ALU_ADD;
                            dec_mem_size_c[i]=funct3[i][1:0];
                            dec_sign_ext_c[i]=funct3[i][2];
                        end
                    OPC_BRANCH:// Branch decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = rs2_raw[i];
                            dec_rd_c[i]  = '0;
                            dec_imm_c[i] = imm_b[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_is_branch_c[i]= 1'b1;
                            dec_has_rd_c[i]  = 1'b0;   
                            dec_funct3_c[i]  = funct3[i]; 
                            dec_unit_sel_c[i]= UNIT_ALU;
                            dec_alu_op_c[i]  = ALU_ADD;
                        end
                    OPC_JAL:   // JAL decode
                        begin
                            dec_rs1_c[i] = '0;
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_j[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_is_jal_c[i]  = 1'b1;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_ALU;
                            dec_alu_op_c[i]  = ALU_ADD;
                        end
                    OPC_JALR:  // JALR decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_j[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_is_jalr_c[i] = 1'b1;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_ALU;
                            dec_funct3_c[i] = 3'b000;
                            dec_alu_op_c[i]  = ALU_ADD;
                        end
                    OPC_LUI:   // LUI decode
                        begin
                            dec_rs1_c[i] = '0;
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_u[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_ALU;
                            dec_alu_op_c[i]  = ALU_LUI;
                        end
                    OPC_AUIPC: // AUIPC decode
                        begin
                            dec_rs1_c[i] = '0;
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_u[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_ALU;
                            dec_alu_op_c[i]  = ALU_AUIPC;
                        end
                    OPC_IMM32: // Word immediate decode
                        begin
                            dec_rs1_c[i] = rs1_raw[i];
                            dec_rs2_c[i] = '0;
                            dec_rd_c[i]  = rd_raw[i];
                            dec_imm_c[i] = imm_i[i];
                            dec_uses_imm_c[i]= 1'b1;
                            dec_has_rd_c[i]  = (rd_raw[i] != '0);    
                            dec_unit_sel_c[i]= UNIT_ALU;
                            case(funct3[i])
                                F3_ADD_SUB : dec_alu_op_c[i] = ALU_ADD;
                                F3_SLL     : dec_alu_op_c[i] = ALU_SLL;
                                F3_SRL_SRA : dec_alu_op_c[i] = funct7[i][5] ? ALU_SRA : ALU_SRL;
                                default    : dec_illegal_c[i]= 1'b1;
                            endcase
                        end
                    OPC_REG32: // Word register decode
                            begin
                                dec_rs1_c[i] = rs1_raw[i];
                                dec_rs2_c[i] = rs2_raw[i];
                                dec_rd_c[i]  = rd_raw[i];
                                dec_imm_c[i] = '0;
                                dec_uses_imm_c[i]= '0;
                                dec_is_word_c[i] = 1'b1;
                                dec_has_rd_c[i]  = (rd_raw[i] != '0);   
                                dec_funct3_c[i] = funct3[i];                        
                                case(funct7[i])
                                    F7_NORMAL:
                                        begin
                                            dec_unit_sel_c[i] = UNIT_ALU;
                                            dec_alu_op_c[i]   = {1'b0, funct3[i]};
                                        end
                                    F7_ALT:
                                        begin
                                            dec_unit_sel_c[i] = UNIT_ALU;
                                            case(funct3[i])
                                                3'b000: dec_alu_op_c[i] = ALU_SUB;
                                                3'b101: dec_alu_op_c[i] = ALU_SRA;
                                                default: dec_illegal_c[i] = 1'b1;
                                            endcase
                                        end
                                    F7_MEXT:
                                        begin
                                            dec_unit_sel_c[i] = UNIT_MULDIV;
                                            dec_alu_op_c[i]   = {1'b0, funct3[i]};
                                            dec_is_mul_c[i]   = !funct3[i][2];       // funct3[2]=0 means MUL variant
                                            dec_is_div_c[i]   =  funct3[i][2];       // funct3[2]=1 means DIV variant
                                        end
                                     default: dec_illegal_c[i] = 1'b1;                                    
                                 endcase
                            end
                    default:   dec_illegal_c[i] = 1'b1;
                endcase
            end
        end 
    end    
    
endmodule