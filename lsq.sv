`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 16.07.2026 23:36:06
// Design Name: RV64G 2-Wide Superscalar OoO Processor
// Module Name: lsq
// Project Name: RV64G 2-Wide Superscalar OoO Processor
// Target Devices: Xilinx 7-Series / UltraScale (Simulation & Synthesis)
// Tool Versions: Vivado 2024.1, xsim 2024.1
// Description: Load/Store Queue (LSQ) with 16-entry Load Queue (LDQ) and 16-entry
//              Store Queue (STQ). Allocates entries at dispatch, fills effective
//              addresses and store data when the ld_st_unit computes them. For loads,
//              performs a CAM search of the STQ for store-to-load forwarding; if no
//              forwarding is possible, issues an L1 D-cache read request (2-cycle
//              response latency). Detects memory ordering violations (a younger load
//              that executed before an older store to the same address) and raises
//              mem_order_violation to trigger a ROB flush. Drains committed stores
//              to the L1 D-cache write port. Returns load results (data + physical
//              tag) to the PRF and ROB via ld_result interface.
//
// Dependencies: (none — internal queues)
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   commit_ldq_idx and commit_stq_idx both connect to commit_lsq_idx in ooo_core
//   because ROB stores a single lsq_idx field used for both load and store commit.
//   mem_size encoding: 00=byte 01=half 10=word 11=double.
//
//////////////////////////////////////////////////////////////////////////////////


module lsq #(
    parameter LDQ_ENTRIES  = 16,  // number of load queue slots
    parameter STQ_ENTRIES  = 16,  // number of store queue slots
    parameter DATA_W       = 64,  // data width for address and data
    parameter ROB_IDX_W    = 6,   // bit width of ROB index
    parameter PHYS_TAG_W   = 7,   // bit width of physical register tag
    parameter ISSUE_WIDTH  = 2    // instructions dispatched per cycle
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // dispatch interface - allocate LDQ or STQ entry at dispatch
    input  logic [ISSUE_WIDTH-1:0]   dispatch_en,                          // dispatch enable per slot
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_load,                     // load instruction flag per slot
    input  logic [ISSUE_WIDTH-1:0]   dispatch_is_store,                    // store instruction flag per slot
    input  logic [ROB_IDX_W-1:0]     dispatch_rob_idx   [0:ISSUE_WIDTH-1], // ROB slot for ordering
    input  logic [PHYS_TAG_W-1:0]    dispatch_phys_dst  [0:ISSUE_WIDTH-1], // physical dst tag for load result
    input  logic [1:0]               dispatch_mem_size  [0:ISSUE_WIDTH-1], // 00=byte 01=half 10=word 11=double
    input  logic [ISSUE_WIDTH-1:0]   dispatch_sign_ext,                    // sign extend load result when high

    // LDQ and STQ slot indices returned to dispatch stage for ROB entry storage
    output logic [4:0]               ldq_idx_out        [0:ISSUE_WIDTH-1], // allocated LDQ slot per slot
    output logic [4:0]               stq_idx_out        [0:ISSUE_WIDTH-1], // allocated STQ slot per slot

    // load execute interface - address computed by load store unit
    input  logic                     ld_exec_valid,      // load has computed effective address this cycle
    input  logic [4:0]               ld_exec_idx,        // which LDQ entry to update
    input  logic [DATA_W-1:0]        ld_exec_addr,       // computed effective address

    // store execute interface - address and data computed by load store unit
    input  logic                     st_exec_valid,      // store has computed address and data this cycle
    input  logic [4:0]               st_exec_idx,        // which STQ entry to update
    input  logic [DATA_W-1:0]        st_exec_addr,       // computed effective address
    input  logic [DATA_W-1:0]        st_exec_data,       // store data value

    // L1 cache interface - load reads and committed store drains
    output logic                     l1_rd_req,          // load cache read request this cycle
    output logic [DATA_W-1:0]        l1_rd_addr,         // effective address for cache read
    input  logic                     l1_rd_valid,        // cache has returned valid data this cycle
    input  logic [DATA_W-1:0]        l1_rd_data,         // data returned from cache
    output logic                     l1_wr_req,          // committed store drain request this cycle
    output logic [DATA_W-1:0]        l1_wr_addr,         // store address being drained to cache
    output logic [DATA_W-1:0]        l1_wr_data,         // store data being drained to cache
    output logic [1:0]               l1_wr_size,         // store access size being drained

    // commit interface - signals from ROB commit stage
    input  logic [ISSUE_WIDTH-1:0]   commit_is_load,                       // load committing this cycle
    input  logic [ISSUE_WIDTH-1:0]   commit_is_store,                      // store committing this cycle
    input  logic [4:0]               commit_ldq_idx     [0:ISSUE_WIDTH-1], // LDQ slot of committing load
    input  logic [4:0]               commit_stq_idx     [0:ISSUE_WIDTH-1], // STQ slot of committing store

    // load result writeback - to PRF and ROB execute complete interface
    output logic                     ld_result_valid,    // load result ready to write to PRF this cycle
    output logic [PHYS_TAG_W-1:0]    ld_result_tag,      // physical destination tag for PRF write
    output logic [DATA_W-1:0]        ld_result_data,     // sign or zero extended load result
    output logic [ROB_IDX_W-1:0]     ld_result_rob_idx,  // ROB index to set complete in ROB

    // load replay - load must re-execute next cycle
    output logic                     ld_replay,          // older store unknown or forwarding data not ready

    // memory ordering violation - younger load got stale data
    output logic                     mem_order_violation, // ordering violation detected this cycle
    output logic [ROB_IDX_W-1:0]     violation_rob_idx,  // ROB index of load that read stale data

    // flush interface
    input  logic                     flush,              // global flush squashes uncommitted entries

    // status
    output logic                     ldq_full,           // LDQ has no free slots
    output logic                     stq_full            // STQ has no free slots
);

    // local parameters
    localparam LDQ_IDX_W = $clog2(LDQ_ENTRIES); // bit width to index one LDQ slot
    localparam STQ_IDX_W = $clog2(STQ_ENTRIES); // bit width to index one STQ slot

    // load queue entry - tracks one in-flight load from dispatch to commit
    typedef struct packed {
        logic [DATA_W-1:0]      addr;        // effective address computed at execute
        logic [DATA_W-1:0]      data;        // loaded data from cache or STQ forwarding
        logic [ROB_IDX_W-1:0]   rob_idx;     // ROB slot for program order comparison
        logic [PHYS_TAG_W-1:0]  phys_dst;    // physical destination tag for PRF write
        logic [1:0]             mem_size;    // access size byte half word double
        logic                   sign_ext;    // sign extend result when high
        logic                   addr_valid;  // effective address has been computed
        logic                   data_valid;  // data field holds valid result
        logic                   executed;    // load completed and result written to PRF
        logic                   forwarded;   // result came from STQ forwarding not cache
        logic                   cache_sent;  // cache read request has been issued
        logic                   valid;       // entry is occupied
    } ldq_entry_t;

    // store queue entry - tracks one in-flight store from dispatch to cache drain
    typedef struct packed {
        logic [DATA_W-1:0]      addr;       // effective address computed at execute
        logic [DATA_W-1:0]      data;       // store data value written at execute
        logic [ROB_IDX_W-1:0]   rob_idx;    // ROB slot for program order comparison with loads
        logic [1:0]             mem_size;   // access size byte half word double
        logic                   addr_valid; // effective address has been computed
        logic                   data_valid; // store data has been computed
        logic                   committed;  // store has committed from ROB and may drain to cache
        logic                   valid;      // entry is occupied by a dispatched store
    } stq_entry_t;

    // queue storage arrays
    ldq_entry_t              ldq [0:LDQ_ENTRIES-1]; // load queue indexed by LDQ slot number
    stq_entry_t              stq [0:STQ_ENTRIES-1]; // store queue indexed by STQ slot number

    // LDQ circular FIFO pointers
    logic [LDQ_IDX_W:0]      ldq_head;
    logic [LDQ_IDX_W:0]      ldq_tail;
    logic [LDQ_IDX_W-1:0]    ldq_head_idx;
    logic [LDQ_IDX_W-1:0]    ldq_tail_idx;

    // STQ circular FIFO pointers
    logic [STQ_IDX_W:0]      stq_head;
    logic [STQ_IDX_W:0]      stq_tail;
    logic [STQ_IDX_W-1:0]    stq_head_idx;
    logic [STQ_IDX_W-1:0]    stq_tail_idx;
    logic [STQ_IDX_W:0]      new_stq_tail;
    // dispatch address computation
    logic [LDQ_IDX_W-1:0]    ldq_dispatch_addr [0:ISSUE_WIDTH-1];
    logic [STQ_IDX_W-1:0]    stq_dispatch_addr [0:ISSUE_WIDTH-1];

    // dispatch counts
    logic [1:0]              ldq_dispatch_count;
    logic [1:0]              stq_dispatch_count;

    // commit counts
    logic [1:0]              ldq_commit_count;

    // STQ CAM search signals - computed when load executes
    logic                    is_older     [0:STQ_ENTRIES-1]; // stq[i] older than executing load
    logic                    addr_match   [0:STQ_ENTRIES-1]; // stq[i] address matches load

    // LDQ CAM search signals - computed when store executes
    logic                    is_younger   [0:LDQ_ENTRIES-1]; // ldq[i] younger than executing store
    logic                    viol_match   [0:LDQ_ENTRIES-1]; // ldq[i] executed with matching addr

    // forwarding result signals
    logic                    fwd_valid;        // matching older store found
    logic [DATA_W-1:0]       fwd_data;         // data from youngest matching store
    logic                    fwd_size_ok;      // store size compatible with load size
    logic [STQ_IDX_W-1:0]    fwd_stq_idx;     // index of youngest matching store
    logic                    fwd_partial;      // address match but size incompatible
    logic                    fwd_older_unknown;// older store with unknown address exists
    logic                    fwd_success;      // forwarding completed successfully this cycle
    logic                    fwd_data_ready;   // data in store is valid 
    
    // cache pending state
    logic                    ld_cache_pending;     // load cache request outstanding
    logic [4:0]              ld_cache_pending_idx; // LDQ slot waiting for cache response
    logic                    cache_returned;        // cache response arrived this cycle

    // active LDQ index for result processing
    logic [4:0]              active_ldq_idx;  // LDQ entry producing result this cycle

    // load result sign extension
    logic [DATA_W-1:0]       ld_raw_data;  // raw data before sign extension
    logic [DATA_W-1:0]       ld_ext_data;  // sign or zero extended final result

    // STQ drain control
    logic                    stq_drain_en;  // STQ head committed and ready to drain
    
    // 1. Sequential Block
    always_ff@(posedge clk)
    begin
        if(!rst_n)
        begin
            for(int i = 0; i < LDQ_ENTRIES; i++)
            begin
                ldq[i].addr        <= '0;     
                ldq[i].rob_idx     <= '0; 
                ldq[i].phys_dst    <= '0;
                ldq[i].mem_size    <= '0; 
                ldq[i].sign_ext    <= '0; 
                ldq[i].addr_valid  <= '0;
                ldq[i].data        <= '0;
                ldq[i].data_valid  <= '0;
                ldq[i].forwarded   <= '0;
                ldq[i].cache_sent  <= '0;
                ldq[i].executed    <= '0; 
                ldq[i].valid       <= '0;
            end
            for(int i = 0; i < STQ_ENTRIES; i++)
            begin
                stq[i].addr        <= '0;
                stq[i].data        <= '0;
                stq[i].rob_idx     <= '0;
                stq[i].mem_size    <= '0;
                stq[i].addr_valid  <= '0;
                stq[i].data_valid  <= '0;
                stq[i].committed   <= '0;
                stq[i].valid       <= '0;
            end
            ldq_head <= '0;
            ldq_tail <= '0;
            stq_head <= '0;
            stq_tail <= '0;
            ld_cache_pending <= '0;
            cache_returned <= '0;
        end
        else
        begin
            if(!flush)
            begin
                // Dispatch Blocks
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin: DISPATCH_LDQ
                    if(dispatch_en[i])
                    begin
                        if(dispatch_is_load[i])
                        begin     
                            ldq[ldq_dispatch_addr[i]].rob_idx     <= dispatch_rob_idx[i]; 
                            ldq[ldq_dispatch_addr[i]].phys_dst    <= dispatch_phys_dst[i];
                            ldq[ldq_dispatch_addr[i]].mem_size    <= dispatch_mem_size[i]; 
                            ldq[ldq_dispatch_addr[i]].sign_ext    <= dispatch_sign_ext[i]; 
                            ldq[ldq_dispatch_addr[i]].valid       <= 1'b1;
                        end
                        else if(dispatch_is_store[i])
                        begin
                            stq[stq_dispatch_addr[i]].rob_idx     <= dispatch_rob_idx[i]; 
                            stq[stq_dispatch_addr[i]].mem_size    <= dispatch_mem_size[i]; 
                            stq[stq_dispatch_addr[i]].valid       <= 1'b1;
                        end
                    end
                end
                ldq_tail <= ldq_tail + (LDQ_IDX_W+1)'(ldq_dispatch_count);
                stq_tail <= stq_tail + (STQ_IDX_W+1)'(stq_dispatch_count);
                
                // Execute Block
                if(ld_exec_valid)
                begin
                    ldq[ld_exec_idx].addr <= ld_exec_addr;
                    ldq[ld_exec_idx].addr_valid <= 1'b1;
                end
                if(st_exec_valid)
                begin
                    stq[st_exec_idx].addr <= st_exec_addr;
                    stq[st_exec_idx].addr_valid <= 1'b1;
                    stq[st_exec_idx].data <= st_exec_data;
                    stq[st_exec_idx].data_valid <= 1'b1;
                end
                if(fwd_success)
                begin
                    ldq[active_ldq_idx].data       <= ld_ext_data;
                    ldq[active_ldq_idx].data_valid <= 1'b1;
                    ldq[active_ldq_idx].executed   <= 1'b1;
                    ldq[active_ldq_idx].forwarded  <= fwd_success;
                end

                // Cache Interface Block — save pending idx only on the first execution
                // cycle (when ld_exec_valid=1), NOT on req_active hold cycles where
                // ld_exec_idx is stale and would corrupt ld_cache_pending_idx.
                if(ld_exec_valid && !fwd_success && !fwd_older_unknown && !fwd_partial)
                begin
                    ld_cache_pending <= 1'b1;
                    ld_cache_pending_idx <= ld_exec_idx;
                    ldq[ld_exec_idx].cache_sent <= 1'b1;
                end
                if(l1_rd_valid)
                begin
                    ld_cache_pending <= 1'b0;
                    cache_returned   <= 1'b1;
                    ldq[ld_cache_pending_idx].data <= l1_rd_data;
                    ldq[ld_cache_pending_idx].data_valid <= 1'b1;
                    ldq[ld_cache_pending_idx].executed <= 1'b1;
                end
                else
                    cache_returned <= 1'b0;
                    
                if(stq_drain_en)
                begin
                    stq[stq_head_idx].valid <= 1'b0;
                    stq_head <= stq_head + (STQ_IDX_W+1)'(1);
                end
                
                // Commit Interface
                for(int i = 0; i < ISSUE_WIDTH; i++)
                begin
                    if(commit_is_load[i])
                    begin
                        ldq[commit_ldq_idx[i]].valid <= 1'b0;
                    end
                    if(commit_is_store[i])
                    begin
                        stq[commit_stq_idx[i]].committed <= 1'b1;
                    end
                end
                ldq_head <= ldq_head + (LDQ_IDX_W+1)'(ldq_commit_count);
            end
            else
            begin
                for(int i = 0; i < LDQ_ENTRIES; i++)
                begin
                    ldq[i].valid <= 1'b0;
                end
                
                for(int i = 0; i < STQ_ENTRIES; i++)
                begin
                    if(!stq[i].committed)
                        stq[i].valid <= 1'b0;
                end
                ldq_tail <= ldq_head;
                stq_tail <= new_stq_tail;
            end
        end
    end
    
    // ------------------------------------------------------------------------
    // FIX: D-Cache Read Request Persistent State (Fixes A6)
    // ------------------------------------------------------------------------
    logic req_active;
    logic [DATA_W-1:0] saved_rd_addr;
    logic l1_rd_req_d; // registered single-cycle pulse: avoids pre/post-NBA mismatch with A6 assertion

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            req_active    <= 1'b0;
            saved_rd_addr <= '0;
            l1_rd_req_d   <= 1'b0;
        end else begin
            // 1. Initiate request if valid load from AGU and no forwarding possible
            if (ld_exec_valid && !fwd_success && !fwd_older_unknown && !fwd_partial && !req_active) begin
                req_active    <= 1'b1;
                saved_rd_addr <= ld_exec_addr;
                l1_rd_req_d   <= 1'b1;
            end
            // 2. Clear request ONLY when cache successfully responds
            else if (req_active && l1_rd_valid) begin
                req_active  <= 1'b0;
                l1_rd_req_d <= 1'b0;
            end else begin
                l1_rd_req_d <= 1'b0;
            end
        end
    end
    
    // 2. Combinational Block
    always_comb
    begin
        ldq_head_idx = ldq_head[LDQ_IDX_W-1:0];
        ldq_tail_idx = ldq_tail[LDQ_IDX_W-1:0];
        stq_head_idx = stq_head[STQ_IDX_W-1:0];
        stq_tail_idx = stq_tail[STQ_IDX_W-1:0];
        ldq_full = (ldq_head_idx == ldq_tail_idx) & (ldq_head[LDQ_IDX_W] != ldq_tail[LDQ_IDX_W]);
        stq_full = (stq_head_idx == stq_tail_idx) & (stq_head[STQ_IDX_W] != stq_tail[STQ_IDX_W]);
        
        ldq_dispatch_count = '0;
        stq_dispatch_count = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            ldq_dispatch_addr[i] = LDQ_IDX_W'(ldq_tail_idx + LDQ_IDX_W'(ldq_dispatch_count));
            stq_dispatch_addr[i] = STQ_IDX_W'(stq_tail_idx + STQ_IDX_W'(stq_dispatch_count));
            if(dispatch_is_load[i] && dispatch_en[i] && (!ldq_full))
                ldq_dispatch_count += 1'd1;
            else if(dispatch_is_store[i] && dispatch_en[i] && (!stq_full))
                stq_dispatch_count += 1'd1;
        end
        
        for(int i = 0; i < ISSUE_WIDTH; i++)
        begin
            ldq_idx_out[i] = ldq_dispatch_addr[i];
            stq_idx_out[i] = stq_dispatch_addr[i];
        end
        
        for(int i = 0; i < STQ_ENTRIES; i++)
        begin
            is_older[i]   = 1'b0;
            addr_match[i] = 1'b0;
        end
        for(int i = 0; i < LDQ_ENTRIES; i++)
        begin
            is_younger[i] = 1'b0;
            viol_match[i] = 1'b0;
        end
        
        fwd_older_unknown = 1'b0;
        fwd_valid   = 1'b0;
        fwd_stq_idx = '0;
        fwd_data    = '0;
        fwd_size_ok = '0;
        fwd_data_ready = '0;
        active_ldq_idx = ld_exec_valid ? ld_exec_idx :(cache_returned ? ld_cache_pending_idx:'0);
        if(ld_exec_valid)
        begin
            for(int i = 0; i < STQ_ENTRIES; i++)
            begin
                is_older[i] = stq[i].valid & (stq[i].rob_idx < ldq[ld_exec_idx].rob_idx);
                addr_match[i] = is_older[i] & stq[i].addr_valid & (stq[i].addr == ld_exec_addr);     
                if(is_older[i] && (!stq[i].addr_valid))
                    fwd_older_unknown = 1'b1;
            end
            for(int i = 0; i < STQ_ENTRIES; i++)
            begin
                if(addr_match[i])
                begin
                    if(!fwd_valid || (stq[i].rob_idx > stq[fwd_stq_idx].rob_idx))
                    begin
                        fwd_valid = 1'b1;
                        fwd_stq_idx = STQ_IDX_W'(i);
                    end
                end
            end
            fwd_data = stq[fwd_stq_idx].data; 
            fwd_size_ok = fwd_valid & (stq[fwd_stq_idx].mem_size >= ldq[ld_exec_idx].mem_size);
            fwd_partial = fwd_valid & !fwd_size_ok;
            fwd_data_ready = stq[fwd_stq_idx].data_valid & fwd_size_ok & fwd_valid;
        end
        ld_replay = (fwd_older_unknown | fwd_partial | (fwd_valid & !fwd_data_ready)) & ld_exec_valid;
        
        l1_rd_req = '0;  
        l1_rd_addr = '0; 
        l1_wr_req = '0;  
        l1_wr_addr = '0; 
        l1_wr_data = '0; 
        l1_wr_size = '0;
        
        fwd_success = ld_exec_valid && fwd_valid && fwd_size_ok && fwd_data_ready && !fwd_older_unknown;
        ld_ext_data = '0;
        ld_raw_data = '0;
        if(fwd_success)
            ld_raw_data = fwd_data;
        else if(cache_returned)
            ld_raw_data = l1_rd_data;
        else
            ld_raw_data = '0;
        if(!ld_replay)
        begin
            case(ldq[active_ldq_idx].mem_size)
                2'b00:  // byte
                    ld_ext_data = ldq[active_ldq_idx].sign_ext ? {{56{ld_raw_data[7]}},  ld_raw_data[7:0]} : {{56{1'b0}}, ld_raw_data[7:0]};
                2'b01:  // halfword
                    ld_ext_data = ldq[active_ldq_idx].sign_ext ? {{48{ld_raw_data[15]}}, ld_raw_data[15:0]}: {{48{1'b0}}, ld_raw_data[15:0]};
                2'b10:  // word
                    ld_ext_data = ldq[active_ldq_idx].sign_ext ? {{32{ld_raw_data[31]}}, ld_raw_data[31:0]}: {{32{1'b0}}, ld_raw_data[31:0]};
                2'b11:  // doubleword
                    ld_ext_data = ld_raw_data;  // no extension needed
                default:
                    ld_ext_data = ld_raw_data;
            endcase 
        end
        ld_result_valid   = fwd_success | cache_returned;
        ld_result_tag     = ldq[active_ldq_idx].phys_dst;
        ld_result_data    = ld_ext_data;
        ld_result_rob_idx = ldq[active_ldq_idx].rob_idx;
        
        mem_order_violation = 1'b0;
        violation_rob_idx   = '0;
        if(st_exec_valid)
        begin
            for(int i = 0; i < LDQ_ENTRIES; i++)
            begin
                is_younger[i] = ldq[i].valid & (ldq[i].rob_idx > stq[st_exec_idx].rob_idx);
                viol_match[i] = is_younger[i] & ldq[i].addr_valid & (ldq[i].addr == stq[st_exec_idx].addr) & ldq[i].executed;
                if(viol_match[i])
                begin
                    if(!mem_order_violation || (ldq[i].rob_idx < violation_rob_idx))
                    begin
                        mem_order_violation = 1'b1;
                        violation_rob_idx = ldq[i].rob_idx;
                    end
                end
            end
        end
        
        // Cache Interface
        l1_rd_req  = l1_rd_req_d;
        l1_rd_addr = req_active ? saved_rd_addr : ld_exec_addr;
        
        stq_drain_en = (rst_n === 1'b1) ? (stq[stq_head_idx].valid & stq[stq_head_idx].committed & !(stq_head == stq_tail)) : 1'b0;
        l1_wr_req  = stq_drain_en;
        l1_wr_addr = stq[stq_head_idx].addr;
        l1_wr_data = stq[stq_head_idx].data;
        l1_wr_size = stq[stq_head_idx].mem_size;
        
        ldq_commit_count = '0;
        for(int i = 0; i < ISSUE_WIDTH; i++)
            ldq_commit_count += commit_is_load[i];
        new_stq_tail = stq_head;
        
        for(int i = 0; i < STQ_ENTRIES; i++)
        begin
            if(stq[i].committed && stq[i].valid)
                new_stq_tail = new_stq_tail + (STQ_IDX_W+1)'(1);
        end
    end
        
endmodule