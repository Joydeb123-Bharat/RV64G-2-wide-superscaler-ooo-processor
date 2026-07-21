`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 17.07.2026 21:58:04
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: branch_predictor
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Hybrid branch predictor with four components:
//              (1) 12-bit Global History Register (GHR).
//              (2) 4096-entry Pattern History Table (PHT) of 2-bit saturating
//                  counters, indexed by GHR XOR PC[13:2].
//              (3) 512-entry Branch Target Buffer (BTB), direct-mapped, indexed
//                  by PC[10:2], storing the predicted target address.
//              (4) 16-entry Return Address Stack (RAS) for call/return prediction.
//              Predicts taken/not-taken and target PC per fetch group. Asserts
//              bp_redirect and bp_target when a taken prediction is made. Saves
//              GHR checkpoints (indexed by ROB slot) at dispatch for recovery.
//              On flush, restores GHR from the checkpoint at flush_rob_idx.
//              Updates PHT and BTB on branch resolution from branch_unit.
//
// Dependencies: (none — internal arrays)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   PHT_ENTRIES=4096=2^GHR_W. BTB_ENTRIES=512. RAS_ENTRIES=16.
//   Call detected as JAL rd!=x0; return detected as JALR rd==x0 rs1==x1.
//
//////////////////////////////////////////////////////////////////////////////////

module branch_predictor #(
    parameter DATA_W      = 64,   // PC width
    parameter GHR_W       = 12,   // global history register width
    parameter PHT_ENTRIES = 4096, // number of PHT entries 2^GHR_W
    parameter BTB_ENTRIES = 512,  // number of BTB entries
    parameter RAS_ENTRIES = 16,   // return address stack depth
    parameter ROB_ENTRIES = 64,   // number of ROB slots for GHR checkpointing
    parameter ROB_IDX_W   = 6,    // bit width of ROB index
    parameter ISSUE_WIDTH = 2     // instructions fetched per cycle
)(
    input  logic                      clk,
    input  logic                      rst_n,

    // fetch interface - predict for current fetch group
    input  logic [DATA_W-1:0]         fetch_pc,                           // current fetch PC
    input  logic [31:0]               fetch_instr    [0:ISSUE_WIDTH-1],   // fetched instruction words
    input  logic [ISSUE_WIDTH-1:0]    fetch_valid,                        // valid slot flags

    // prediction outputs - to fetch unit and dispatch stage same cycle
    output logic                      bp_redirect,                        // predict taken redirect
    output logic [DATA_W-1:0]         bp_target,                          // predicted target PC
    output logic [ISSUE_WIDTH-1:0]    predicted_taken,                    // per slot direction
    output logic [DATA_W-1:0]         predicted_target [0:ISSUE_WIDTH-1], // per slot target
    output logic [GHR_W-1:0]          current_ghr,                        // current GHR for dispatch snapshot

    // GHR checkpoint interface - save at dispatch for each branch
    input  logic                      ckpt_en,                            // save GHR checkpoint
    input  logic [ROB_IDX_W-1:0]      ckpt_rob_idx,                       // ROB slot for checkpoint

    // update interface - actual outcome from branch unit
    input  logic                      update_en,                          // branch resolved
    input  logic [DATA_W-1:0]         update_pc,                          // PC of resolved branch
    input  logic                      update_taken,                       // actual taken outcome
    input  logic [DATA_W-1:0]         update_target,                      // actual target address
    input  logic [GHR_W-1:0]          update_ghr,                         // GHR at fetch time for PHT index
    input  logic                      update_is_call,                     // resolved branch was JAL
    input  logic                      update_is_ret,                      // resolved branch was JALR

    // flush interface - restore GHR from checkpoint
    input  logic                      flush,                              // misprediction detected
    input  logic [ROB_IDX_W-1:0]      flush_rob_idx                       // ROB index of mispredicted branch
);

    // local parameters
    localparam PHT_IDX_W = $clog2(PHT_ENTRIES);  // bit width to index PHT
    localparam BTB_IDX_W = $clog2(BTB_ENTRIES);  // bit width to index BTB
    localparam RAS_IDX_W = $clog2(RAS_ENTRIES);  // bit width to index RAS

    // GHR
    logic [GHR_W-1:0]        ghr;                          // current speculative GHR

    // GHR checkpoint array
    logic [GHR_W-1:0]        ghr_ckpt [0:ROB_ENTRIES-1];  // GHR snapshots per ROB slot

    // PHT
    logic [1:0]              pht [0:PHT_ENTRIES-1];        // 2-bit saturating counters

    // BTB entry struct
    typedef struct packed {
        logic [DATA_W-1:0]   target;   // last observed branch target
        logic [DATA_W-1:0]   tag;      // full PC tag for hit detection
        logic                valid;    // entry is valid
    } btb_entry_t;

    btb_entry_t              btb [0:BTB_ENTRIES-1];        // BTB indexed by PC bits

    // RAS
    logic [DATA_W-1:0]       ras     [0:RAS_ENTRIES-1];   // return address stack
    logic [RAS_IDX_W-1:0]    ras_top;                     // top of stack pointer

    // per slot PC computation
    logic [DATA_W-1:0]       slot_pc [0:ISSUE_WIDTH-1];   // PC of each slot in fetch group

    // per slot index computation
    logic [PHT_IDX_W-1:0]    pht_idx [0:ISSUE_WIDTH-1];   // PHT index per slot
    logic [BTB_IDX_W-1:0]    btb_idx [0:ISSUE_WIDTH-1];   // BTB index per slot

    // per slot classification
    logic [ISSUE_WIDTH-1:0]  is_branch;   // slot is a conditional branch
    logic [ISSUE_WIDTH-1:0]  is_call;     // slot is a JAL call
    logic [ISSUE_WIDTH-1:0]  is_ret;      // slot is a JALR return
    logic [ISSUE_WIDTH-1:0]  btb_hit;     // BTB has valid matching entry for slot
    logic [ISSUE_WIDTH-1:0]  pht_taken;   // PHT predicts taken for slot

    // update index computation
    logic [PHT_IDX_W-1:0]    update_pht_idx;  // PHT index for branch being updated
    logic [BTB_IDX_W-1:0]    update_btb_idx;  // BTB index for branch being updated
    
    // 1. Sequentail Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            ghr <= '0;
            for(int i = 0; i < BTB_ENTRIES; i++)
            begin
                btb[i].target <= '0;
                btb[i].tag <= '0;
                btb[i].valid <= '0;
            end
            
            for(int i = 0; i < RAS_ENTRIES; i++)
                ras[i] <= '0;
            ras_top <= '0;
            
            for(int i = 0; i < PHT_ENTRIES; i++)
                pht[i] <= 2'b01;
            
            for(int i = 0; i < ROB_ENTRIES; i++)
                ghr_ckpt[i] <= '0;
                
        end
        else
        begin
            if(!flush)
            begin
                // Check Point
                if(ckpt_en)
                    ghr_ckpt[ckpt_rob_idx] <= ghr;
                
                
                for(int  i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(is_call[i] && fetch_valid[i] && (ras_top != '1)) // For JA:
                    begin
                        ras[ras_top] <= slot_pc[i] + DATA_W'(4);    
                        ras_top <= ras_top + 1'd1;
                    end
                    else if(is_ret[i] && fetch_valid[i] && (ras_top != '0)) // For JALR
                        ras_top <= ras_top - RAS_IDX_W'(1);
                    
                    if((is_branch[i] || is_call[i] || is_ret[i]) && fetch_valid[i]) // For GHR update
                    begin
                        ghr <= { ghr[GHR_W-1:1], predicted_taken[i] };
                        break;
                    end    
                end 
                
                // Update Interface
                if(update_en)
                begin
                    if(!update_is_ret)
                    begin
                        if(update_taken)
                            pht[update_pht_idx] <= (pht[update_pht_idx] == 2'b11) ? 2'b11 : pht[update_pht_idx] + 1'd1;
                        else
                            pht[update_pht_idx] <= (pht[update_pht_idx] == 2'b00) ? 2'b00 : pht[update_pht_idx] - 1'd1;
                        if(update_taken)
                        begin
                            btb[update_btb_idx].valid <= 1'b1;
                            btb[update_btb_idx].target <= update_target;
                            btb[update_btb_idx].tag <= update_pc;
                        end
                    end
                end            
                    
            end
            else
            begin
                ghr <= {ghr_ckpt[flush_rob_idx][GHR_W-2:0], update_taken};
            end
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        current_ghr = ghr;
        bp_target   = '0;
        bp_redirect = '0;
        predicted_taken = '0;
        
       for(int  i = 0; i < ISSUE_WIDTH; i++)
       begin
            
            is_branch[i] = (fetch_instr[i][6:0] == 7'b1100011);
            is_call[i]   = (fetch_instr[i][6:0] == 7'b1101111);
            is_ret[i]    = (fetch_instr[i][6:0] == 7'b1100111);
            slot_pc[i]   = fetch_pc + DATA_W'(3'd4 * (i));
            
            pht_idx[i] = ghr ^ slot_pc[i][PHT_IDX_W+1:2];
            btb_idx[i] = slot_pc[i][BTB_IDX_W+1:2];
            pht_taken[i]  = pht[pht_idx[i]][1];
            btb_hit[i]    = btb[btb_idx[i]].valid & (btb[btb_idx[i]].tag == slot_pc[i]);
            
            if((is_branch[i]||is_call[i]) && fetch_valid[i] && !flush && pht_taken[i] && btb_hit[i] && !bp_redirect)
            begin
                bp_target = btb[btb_idx[i]].target;
                bp_redirect = 1'b1;
                predicted_taken[i] = 1'b1;
            end
            else if(is_ret[i] && fetch_valid[i] && !flush )
            begin                 
               bp_target = (ras_top != '0) ? ras[RAS_IDX_W'(ras_top-1)] : fetch_pc + DATA_W'(8);
               bp_redirect = (ras_top != '0);
               predicted_taken[i] = (ras_top != '0);
            end
            else
                bp_target = fetch_pc + DATA_W'(8);
            predicted_target[i] = bp_target;
            
        end
        
        
        
        update_pht_idx = update_pc[PHT_IDX_W+1:2] ^ update_ghr;
        update_btb_idx = update_pc[BTB_IDX_W+1:2];
    end
endmodule