`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 19:56:43
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: div_unit
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Iterative non-restoring integer divider for the RV64G M-extension.
//              Supports DIV (signed quotient), DIVU (unsigned quotient), REM (signed
//              remainder), REMU (unsigned remainder), and the corresponding 32-bit
//              W-suffix variants (DIVW, DIVUW, REMW, REMUW). FSM states: IDLE →
//              RUNNING → DONE (normal) or DIV_SPECIAL (divide-by-zero / overflow).
//              Takes up to 32 cycles for 32-bit and 64 cycles for 64-bit operands.
//              Asserts unit_available=0 while RUNNING, blocking IQ from issuing
//              another divide. Handles RISC-V special cases: x÷0 returns -1 (quotient)
//              and x (remainder); INT_MIN÷-1 returns INT_MIN (overflow).
//
// Dependencies: (none — FSM datapath)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   div_op_in encoding: 2'b00=DIV 2'b01=DIVU 2'b10=REM 2'b11=REMU.
//   Shares MULDIV issue slot (issue_valid[2]) with mul_unit; gated by is_div flag.
//
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module div_unit #(
    parameter DATA_W     = 64,
    parameter ROB_IDX_W  = 6,
    parameter PHYS_TAG_W = 7
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // flush interface
    input  logic                   flush,

    // input interface - from issue queue
    input  logic                   valid_in,
    input  logic [DATA_W-1:0]      src1_in,
    input  logic [DATA_W-1:0]      src2_in,
    input  logic [1:0]             div_op_in,
    input  logic                   is_word_in,
    input  logic [ROB_IDX_W-1:0]   rob_idx_in,
    input  logic [PHYS_TAG_W-1:0]  phys_dst_in,

    // output interface - result and ROB writeback
    output logic                   valid_out,
    output logic                   complete_out,
    output logic                   mispredicted_out,
    output logic                   exception_out,
    output logic [3:0]             exc_cause_out,
    output logic [DATA_W-1:0]      branch_target_out,
    output logic [DATA_W-1:0]      result_out,
    output logic [ROB_IDX_W-1:0]   rob_idx_out,
    output logic [PHYS_TAG_W-1:0]  phys_dst_out,

    // unit availability
    output logic                   unit_available
);

    // local parameters - operation encoding
    localparam DIV  = 2'd0;  // signed quotient
    localparam DIVU = 2'd1;  // unsigned quotient
    localparam REM  = 2'd2;  // signed remainder
    localparam REMU = 2'd3;  // unsigned remainder

    // state machine encoding
    localparam IDLE        = 2'd0;  // waiting for valid instruction
    localparam RUNNING     = 2'd1;  // iterating through division cycles
    localparam DIV_SPECIAL = 2'd2;  // division by zero or overflow result pre-computed
    localparam DONE        = 2'd3;  // result ready assert complete for one cycle

    // iteration counter width
    localparam CTR_W = $clog2(DATA_W + 1);  // wide enough to count 0 to DATA_W

    // state register
    logic [1:0]              state;           // current FSM state

    // iteration counter and limit
    logic [CTR_W-1:0]        counter;         // counts iterations 0 to iter_limit
    logic [CTR_W-1:0]        iter_limit;      // 31 for word operations 63 for doubleword

    // stored instruction fields captured at start of division
    logic [ROB_IDX_W-1:0]    rob_idx_reg;     // ROB slot held during division
    logic [PHYS_TAG_W-1:0]   phys_dst_reg;    // physical destination held during division
    logic [1:0]              div_op_reg;      // operation type held during division
    logic                    is_word_reg;     // word flag held during division

    // operand registers - unsigned versions used during iteration
    logic [DATA_W-1:0]       dividend_reg;    // unsigned dividend captured at start
    logic [DATA_W-1:0]       divisor_reg;     // unsigned divisor captured at start

    // sign correction flags captured at start from original signed operands
    logic                    dividend_neg;    // dividend was negative before unsigned conversion
    logic                    divisor_neg;     // divisor was negative before unsigned conversion

    // special case flags
    logic                    div_by_zero;     // asserted when divisor is zero
    logic                    overflow;        // asserted when dividend=MIN_INT and divisor=-1

    // working registers for restoring division algorithm
    logic [2*DATA_W-1:0]     remainder;       // 128-bit partial remainder shifted each iteration
    logic [DATA_W-1:0]       quotient;        // accumulates one bit per iteration

    // final iteration combinational signals
    logic [DATA_W-1:0]       final_quotient;    // quotient including current iteration bit
    logic [DATA_W-1:0]       final_remainder;   // remainder after current iteration
    logic [DATA_W-1:0]       final_quot_signed; // sign-corrected final quotient
    logic [DATA_W-1:0]       final_rem_signed;  // sign-corrected final remainder

    // intermediate signals
    logic [DATA_W-1:0]       abs_dividend;    // unsigned dividend computed combinationally at input
    logic [DATA_W-1:0]       abs_divisor;     // unsigned divisor computed combinationally at input
    logic [DATA_W-1:0]       rem_upper;       // upper 64 bits of remainder for clean indexing
    logic [DATA_W:0]         diff;            // DATA_W+1 bits to detect negative result
    logic [2*DATA_W-1:0]     remainder_shifted; // shifted remainder computed combinationally

    // result register
    logic [DATA_W-1:0]       result_reg;      // final quotient or remainder after sign fix

    // 1. Sequential Block
    always_ff @(posedge clk)
    begin
        if(!rst_n)
        begin
            counter      <= '0;
            dividend_neg <= '0;
            divisor_neg  <= '0;
            dividend_reg <= '0;
            divisor_reg  <= '0;
            rob_idx_reg  <= '0;
            phys_dst_reg <= '0;
            div_op_reg   <= '0;
            is_word_reg  <= '0;
            state        <= IDLE;
            result_reg   <= '0;
            quotient     <= '0;
            remainder    <= '0;
            iter_limit   <= '0;
            div_by_zero  <= '0;
            overflow     <= '0;
        end
        else
        begin
            if(flush)
            begin
                state        <= IDLE;
                counter      <= '0;
                quotient     <= '0;
                remainder    <= '0;
                div_by_zero  <= '0;
                overflow     <= '0;
                dividend_neg <= '0;
                divisor_neg  <= '0;
                dividend_reg <= '0;
                divisor_reg  <= '0;
                rob_idx_reg  <= '0;
                phys_dst_reg <= '0;
                div_op_reg   <= '0;
                is_word_reg  <= '0;
                iter_limit   <= '0;
            end
            else
            begin
                // capture instruction fields on valid_in
                if(valid_in)
                begin
                    divisor_neg  <= src2_in[DATA_W-1];
                    dividend_neg <= src1_in[DATA_W-1];
                    is_word_reg  <= is_word_in;
                    rob_idx_reg  <= rob_idx_in;
                    phys_dst_reg <= phys_dst_in;
                    div_op_reg   <= div_op_in;
                end

                // IDLE state
                if(state == IDLE && valid_in)
                begin
                    counter     <= '0;
                    quotient    <= '0;
                    iter_limit  <= is_word_in ? CTR_W'(31) : CTR_W'(63);
                    div_by_zero <= (src2_in == '0);
                    overflow    <= (div_op_in == DIV || div_op_in == REM) &&
                                   (src1_in == {1'b1, {(DATA_W-1){1'b0}}}) &&
                                   (src2_in == '1);
                    dividend_reg <= abs_dividend;
                    divisor_reg  <= abs_divisor;
                    remainder    <= is_word_in ?
                                    {{(DATA_W + DATA_W/2){1'b0}}, abs_dividend[31:0]} :
                                    {{DATA_W{1'b0}}, abs_dividend};
                    if(src2_in == '0)
                        state <= DIV_SPECIAL;
                    else
                        state <= RUNNING;
                end

                // RUNNING state
                else if(state == RUNNING)
                begin
                    if(!diff[DATA_W])
                        remainder <= {diff[DATA_W-1:0], remainder_shifted[DATA_W-1:0]};
                    else
                        remainder <= remainder_shifted;
                    quotient[DATA_W-1-counter] <= ~diff[DATA_W];
                    counter <= counter + CTR_W'(1);
                    if(counter == iter_limit)
                    begin
                        case(div_op_reg)
                            DIV:     result_reg <= is_word_reg ?
                                                   {{32{final_quot_signed[31]}}, final_quot_signed[31:0]} :
                                                   final_quot_signed;
                            DIVU:    result_reg <= is_word_reg ?
                                                   {{32{1'b0}}, final_quotient[31:0]} :
                                                   final_quotient;
                            REM:     result_reg <= is_word_reg ?
                                                   {{32{final_rem_signed[31]}}, final_rem_signed[31:0]} :
                                                   final_rem_signed;
                            REMU:    result_reg <= is_word_reg ?
                                                   {{32{1'b0}}, final_remainder[31:0]} :
                                                   final_remainder;
                            default: result_reg <= '0;
                        endcase
                        state <= DONE;
                    end
                end

                // DIV_SPECIAL state
                else if(state == DIV_SPECIAL)
                begin
                    if(div_by_zero)
                    begin
                        case(div_op_reg)
                            DIV, DIVU: result_reg <= '1;
                            REM, REMU: result_reg <= dividend_reg;
                            default:   result_reg <= '0;
                        endcase
                    end
                    else
                    begin
                        // overflow case - dividend=MIN_INT divisor=-1
                        case(div_op_reg)
                            DIV:     result_reg <= is_word_reg ?
                                                   {{32{src1_in[31]}}, src1_in[31:0]} :
                                                   src1_in;
                            REM:     result_reg <= '0;
                            default: result_reg <= '0;
                        endcase
                    end
                    state <= DONE;
                end

                // DONE state
                else if(state == DONE)
                begin
                    state <= IDLE;
                end
            end
        end
    end

    // 2. Combinational Block
    always_comb
    begin
        // unsigned operand computation from live inputs
        abs_dividend = ((div_op_in == DIV || div_op_in == REM) && src1_in[DATA_W-1]) ?
                       -src1_in : src1_in;
        abs_divisor  = ((div_op_in == DIV || div_op_in == REM) && src2_in[DATA_W-1]) ?
                       -src2_in : src2_in;

        // restoring division algorithm combinational paths
        remainder_shifted = remainder << 1;
        diff = {1'b0, remainder_shifted[2*DATA_W-1:DATA_W]} - {1'b0, divisor_reg};

        // remainder upper half for clean indexing
        rem_upper = remainder[2*DATA_W-1:DATA_W];

        // final iteration values including current cycle bit
        for(int i = 0; i < DATA_W; i++)
        begin
            final_quotient[i] = (i == (DATA_W-1-counter)) ? ~diff[DATA_W] : quotient[i];
        end
        final_remainder   = !diff[DATA_W] ? diff[DATA_W-1:0] : remainder_shifted[2*DATA_W-1:DATA_W];
        final_quot_signed = (dividend_neg ^ divisor_neg) ? -final_quotient : final_quotient;
        final_rem_signed  = dividend_neg ? -final_remainder : final_remainder;

        // output signals
        valid_out         = (state == DONE);
        complete_out      = valid_out;
        mispredicted_out  = 1'b0;
        exception_out     = 1'b0;
        exc_cause_out     = 4'd0;
        branch_target_out = '0;
        rob_idx_out       = rob_idx_reg;
        phys_dst_out      = phys_dst_reg;
        unit_available    = (state == IDLE);
        result_out        = result_reg;
    end

endmodule