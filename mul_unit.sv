`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 16:58:24
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: mul_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: 2-stage pipelined multiply unit for the RV64G M-extension.
//              Supports four operations selected by mul_op_in[1:0]:
//              MUL (lower 64 bits, signed×signed), MULH (upper 64 bits, signed×signed),
//              MULHU (upper 64 bits, unsigned×unsigned), MULHSU (upper 64 bits,
//              signed×unsigned). The is_word_in flag enables 32-bit MULW mode
//              (operands truncated to 32 bits, result sign-extended to 64 bits).
//              Result is valid 2 cycles after valid_in. unit_available is always 1
//              (fully pipelined, accepts a new operation every cycle). Output is
//              suppressed (valid_out=0) if flush is asserted while in-flight.
//
// Dependencies: (none — datapath pipeline)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   N_STAGES=2. Wakeup bus fires on cycle 2 (same cycle as valid_out).
//   mul_op_in encoding matches funct3[1:0]: 00=MUL 01=MULH 10=MULHSU 11=MULHU.
//
//////////////////////////////////////////////////////////////////////////////////

module mul_unit #(
    parameter DATA_W     = 64,  // operand width
    parameter ROB_IDX_W  = 6,   // bit width of ROB index
    parameter PHYS_TAG_W = 7    // bit width of physical register tag
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // flush interface
    input  logic                   flush,           // suppress output on flush to prevent stale PRF writes

    // input interface - from issue queue
    input  logic                   valid_in,        // instruction valid this cycle
    input  logic [DATA_W-1:0]      src1_in,         // first source operand
    input  logic [DATA_W-1:0]      src2_in,         // second source operand
    input  logic [1:0]             mul_op_in,       // operation select MUL MULH MULHU MULHSU
    input  logic                   is_word_in,      // high for MULW operate on lower 32 bits
    input  logic [ROB_IDX_W-1:0]   rob_idx_in,      // ROB slot for writeback
    input  logic [PHYS_TAG_W-1:0]  phys_dst_in,     // physical destination tag

    // output interface - result and wakeup broadcast
    output logic                   valid_out,       // result valid this cycle gated on flush
    output logic                   complete_out,    // asserted when ROB entry should be updated
    output logic                   mispredicted_out,// always 0 multiply never mispredicts
    output logic                   exception_out,   // always 0 multiply never excepts
    output logic [3:0]             exc_cause_out,   // always 0
    output logic [DATA_W-1:0]      branch_target_out, // always 0 multiply has no branch target
    output logic [DATA_W-1:0]      result_out,      // 64-bit multiply result
    output logic [ROB_IDX_W-1:0]   rob_idx_out,     // ROB index for execute complete
    output logic [PHYS_TAG_W-1:0]  phys_dst_out,    // physical tag for PRF write

    // unit availability
    output logic                   unit_available   // always 1 pipelined accepts every cycle
);

    // local parameters
    localparam MUL    = 2'd0;  // lower 64 bits of signed x signed product
    localparam MULH   = 2'd1;  // upper 64 bits of signed x signed product
    localparam MULHU  = 2'd2;  // upper 64 bits of unsigned x unsigned product
    localparam MULHSU = 2'd3;  // upper 64 bits of signed x unsigned product
    localparam N_STAGES = 2;   // three pipeline stages fixed latency

    // pipeline registers
    logic                   valid_pipe   [0:N_STAGES-1]; // valid bit through pipeline
    logic [ROB_IDX_W-1:0]   rob_pipe     [0:N_STAGES-1]; // ROB index through pipeline
    logic [PHYS_TAG_W-1:0]  dst_pipe     [0:N_STAGES-1]; // destination tag through pipeline
    logic [1:0]             op_pipe      [0:N_STAGES-1]; // operation code through pipeline
    logic                   word_pipe    [0:N_STAGES-1]; // is_word flag through pipeline
    logic [2*DATA_W-1:0]    product_pipe;                // 128-bit product through pipeline
    logic [DATA_W-1:0] src1_pipe;                        // registered src1 operand used for product computation
    logic [DATA_W-1:0] src2_pipe;                        // registered src2 operand used for product computation
    
    // stage 1 combinational products
    logic [2*DATA_W-1:0]    product_ss;  // signed x signed 128-bit product
    logic [2*DATA_W-1:0]    product_uu;  // unsigned x unsigned 128-bit product
    logic [2*DATA_W-1:0]    product_su;  // signed x unsigned 128-bit product

    // output stage raw result before word sign extension
    logic [DATA_W-1:0]      raw_result;  // selected 64-bit half before word adjustment
    
    // 1. Sequential Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            for( int i = 0; i < N_STAGES; i++)
            begin
                valid_pipe[i]     <= '0;
                rob_pipe[i]       <= '0;
                dst_pipe[i]       <= '0;
                word_pipe[i]      <= '0;
                product_pipe      <= '0;
                op_pipe[i]        <= '0;
            end
            src1_pipe         <= '0;
            src2_pipe         <= '0;
        end
        else
        begin
            if(!flush)
            begin
                for(int  i = N_STAGES-1; i > 0; i--)
                begin
                    valid_pipe[i]  <= valid_pipe[i-1];  
                    rob_pipe[i]    <= rob_pipe[i-1];    
                    dst_pipe[i]    <= dst_pipe[i-1];    
                    word_pipe[i]   <= word_pipe[i-1];
                    op_pipe[i]     <= op_pipe[i-1];
                end 
                valid_pipe[0]     <= valid_in;
                rob_pipe[0]       <= rob_idx_in;
                dst_pipe[0]       <= phys_dst_in;
                word_pipe[0]      <= is_word_in;
                op_pipe[0]        <= mul_op_in;
                src1_pipe         <= src1_in;
                src2_pipe         <= src2_in;
                case(op_pipe[0])
                    MUL,MULH : product_pipe <= product_ss;
                    MULHU    : product_pipe <= product_uu;
                    MULHSU   : product_pipe <= product_su;
                    default  : product_pipe <= '0;
                endcase
           end
           else
           begin
                for( int i = 0; i < N_STAGES; i++)
                    valid_pipe[i]     <= '0;
           end
        end
    end
    
    // 2. Combinational Blcok
    always_comb
    begin
        unit_available = 1'b1;
        mispredicted_out = 1'b0;
        exception_out = 1'b0;
        exc_cause_out = '0;
        branch_target_out = '0;
        product_ss = $signed(src1_pipe) * $signed(src2_pipe);
        product_uu = src1_pipe * src2_pipe;
        product_su = $signed(src1_pipe) * $signed({1'b0,src2_pipe});
        
        valid_out    = valid_pipe[N_STAGES-1] & (!flush);
        complete_out = valid_out;
        phys_dst_out = dst_pipe[N_STAGES-1];
        rob_idx_out  = rob_pipe[N_STAGES-1];
        
        case(op_pipe[1])
            MUL:     raw_result = product_pipe[63:0];
            MULH:    raw_result = product_pipe[127:64];
            MULHU:   raw_result = product_pipe[127:64];
            MULHSU:  raw_result = product_pipe[127:64];
            default: raw_result = '0;
        endcase
        result_out = word_pipe[1] ? {{32{raw_result[31]}}, raw_result[31:0]} : raw_result;
        
    end
    
endmodule
