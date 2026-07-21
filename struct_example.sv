`timescale 1ns / 1ps

// 1. Define the packed struct outside the module
// The 'packed' keyword ensures this synthesizes into a contiguous 14-bit vector
typedef struct packed {
    logic       valid;      // 1 bit  (MSB)
    logic [4:0] address;    // 5 bits
    logic [7:0] data;       // 8 bits (LSB)
} packet_t;

module struct_example (
    input  logic    clk,
    input  logic    rst_n,
    
    // 2. Use the struct as an input port
    input  packet_t in_packet,
    
    // Outputs
    output logic [7:0] out_data,
    output logic       out_valid
);

    // 3. Access struct members using dot notation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_data  <= 8'h00;
            out_valid <= 1'b0;
        end else begin
            // Check the 'valid' member of the struct
            if (in_packet.valid) begin
                out_data  <= in_packet.data; // Route the data member
                out_valid <= 1'b1;
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

endmodule