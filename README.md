# RV64IM 2-Wide Superscalar Out-of-Order Processor

A fully functional, simulation-verified **RV64IM 2-wide superscalar out-of-order processor** implemented in SystemVerilog, targeting Vivado 2025.2 / XSim. Built as a portfolio project demonstrating advanced computer architecture and RTL design skills at a level competitive for senior roles at Apple Silicon, NVIDIA, AMD, and Qualcomm.

---

## Simulation Results

```
SIMULATION COMPLETE
  PASS: 43
  FAIL: 0
```

All 43 directed test cases pass with zero failures across 15 test programs covering ALU operations, word operations, RAW bypass, branch prediction and misprediction recovery, JAL/JALR, loads and stores, load-use hazards, store-to-load forwarding, MUL, DIV, DIV-by-zero, and superscalar dispatch.

---

## Architecture Overview

<img width="3440" height="2800" alt="rv64im-ooo-block-diagram" src="https://github.com/user-attachments/assets/0bb1c748-b3a4-4ee5-87c3-cac03c3ad2c2" />

### Key Microarchitectural Parameters

| Parameter | Value |
|---|---|
| ISA | RV64IM |
| Issue width | 2-wide superscalar |
| Physical registers | 96 (32 arch + 64 free) |
| ROB entries | 64 |
| Issue queue entries | 16 |
| Load queue entries | 16 |
| Store queue entries | 16 |
| Branch predictor | Gshare, 12-bit GHR, 4096-entry PHT |
| BTB entries | 512 |
| RAS depth | 16 |
| MUL latency | 2 cycles |
| DIV latency | 32–64 cycles (restoring) |
| Reset | Synchronous active-low |

---

## Module Descriptions

### Front-End

| Module | Description |
|---|---|
| `fetch_unit.sv` | PC register with flush/redirect priority mux. Fetches 2 instructions per cycle from I-cache. Manages redirect_pending to discard stale cache responses. |
| `branch_predictor.sv` | Gshare predictor with 12-bit GHR XOR PC indexing into 4096-entry PHT of 2-bit saturating counters. Includes 512-entry BTB and 16-entry RAS. GHR checkpointed per ROB slot for misprediction recovery. |
| `decode_unit.sv` | Fully combinational decode of all RV64IM instruction formats (R/I/S/B/U/J). ALU op codes aligned to funct3 for single-expression decode. Registered output buffer held on stall, cleared on flush. |
| `dispatch_unit.sv` | Purely combinational register renaming. Reads RAT for source tags, pops Free List for destination tags, checks PRF valid bits for initial src_rdy, allocates ROB/IQ/LSQ entries, handles intra-dispatch bypass for slot 1 reading slot 0's new mapping. |

### Rename Infrastructure

| Module | Description |
|---|---|
| `free_list.sv` | Circular FIFO of free physical register tags. Head-pointer checkpointing per ROB slot for flush recovery. Pops blocked during flush; pushes from commit always proceed. |
| `rat.sv` | Speculative front-end Register Alias Table. Identity mapping at reset. Intra-dispatch bypass for same-cycle slot 0→slot 1 forwarding. Restores from retire_rat on flush in one cycle. |
| `retire_rat.sv` | Commit-only RAT updated at retire time. Provides the architectural ground-truth mapping used to restore the speculative RAT on flush. |

### Core Buffers

| Module | Description |
|---|---|
| `rob.sv` | 64-entry circular FIFO Reorder Buffer. Tracks in-flight instructions from dispatch to commit. Generates flush signal with correct target on misprediction or exception. Commits up to 2 instructions per cycle in program order. |
| `issue_queue.sv` | 16-entry collapsing queue with oldest-first selection. CAM-based wakeup broadcast from all execution units. Carries full instruction context including funct3, mem_size, lsq_idx, is_word, ghr_snapshot through to issue. |
| `lsq.sv` | Independent LDQ/STQ circular FIFOs (16 entries each). STQ CAM search at load execute for store-to-load forwarding. LDQ CAM search at store execute for memory ordering violation detection. STQ drains in-order from head after commit. |

### Execution Units

| Module | Description |
|---|---|
| `alu_64.sv` | Purely combinational 64-bit ALU. Op codes aligned to funct3 for direct decode mapping: `{funct7[5], funct3}`. Supports 32-bit word operations with sign extension via `is_word`. |
| `branch_unit.sv` | Purely combinational. Resolves actual taken/target. Detects misprediction by comparing against predicted_taken/predicted_target. Handles JAL/JALR return address write. JALR clears LSB of target. |
| `mul_unit.sv` | 2-stage pipelined multiplier. Supports MUL/MULH/MULHU/MULHSU and word variants. Gated on flush. |
| `div_unit.sv` | 4-state FSM restoring divider (64 iterations, 32 for word operations). Handles division-by-zero and signed overflow corner cases. `unit_available` blocks new issues during execution. |
| `ld_st_unit.sv` | Purely combinational effective address computation (`base + imm`). Checks alignment for byte/half/word/double accesses. Sends address to LSQ; never writes PRF directly. |

### System

| Module | Description |
|---|---|
| `prf.sv` | 96-entry Physical Register File. 8 read ports (2 per execution unit), 4 write ports. Valid bit per entry for src_rdy tracking at dispatch. Invalidated on rename, written by execution units. |
| `csr.sv` | Minimal M-mode CSR: mtvec, mepc, mcause, mtval, mstatus, mcycle, minstret. Exception commit updates mepc/mcause/mtval and saves mstatus. MRET restores mstatus. mcycle increments every clock unconditionally. |
| `ooo_core.sv` | Top-level integration. No logic — instantiates all 17 sub-modules and connects all signals. Resolves all wakeup bus, PRF write bus, and ROB exec-complete bus connections. |

---

## Testbench

`ooo_tb.sv` is a self-checking SystemVerilog testbench providing:

- **I-cache and D-cache memory models** — 16 KB IMEM, 64 KB DMEM with 2-cycle read latency
- **15 directed test programs** encoded as 32-bit instruction arrays covering:
  - Basic ALU operations (ADD SUB AND OR XOR SLL SRL SLT SLTU LUI)
  - Word operations (ADDW SUBW ADDIW)
  - RAW bypass chains
  - Branch taken / not taken / misprediction recovery
  - JAL / JALR with return address
  - Loads and stores (LD LW LBU SD)
  - Load-use hazards
  - Store-to-load forwarding
  - MUL / MULH / MULW
  - DIV / DIVU / REM / REMU
  - Division by zero
  - Superscalar 2-wide dispatch
  - Signed shift and compare (SRA SLT SLTU)
- **13 SVA assertions** covering reset cleanliness, dispatch invariants, ROB ordering, wakeup validity, and cache protocol
- **5 functional coverage groups**: instruction mix, pipeline events, branch outcomes, memory access sizes, address ranges

---

## Key Design Decisions

### ALU Op Code Encoding
ALU op codes are aligned to RISC-V `funct3` values: `dec_alu_op = {funct7[5], funct3}`. This enables single-expression decode for all integer ALU and M-extension instructions — no nested case statements for the common path.

### IQ Collapsing vs Circular FIFO
The issue queue is a collapsing queue where index 0 is always the oldest entry. This eliminates the need for explicit age tracking — oldest-first grant selection is a simple linear scan from index 0. The collapsing shift is computed combinationally from a `grants_below` array.

### Free List Checkpointing
On misprediction, the Free List restores from a head-pointer checkpoint saved at dispatch time for the mispredicted branch's ROB slot. This is O(1) recovery — no ROB walking required.

### Branch Predictor Redirect Rule
`bp_redirect` only asserts when BOTH `pht_taken` AND `btb_hit` are true. A BTB miss on a predicted-taken branch falls back to predict-not-taken — conservative but correct. The first occurrence of a new branch causes at most one misprediction before the BTB is populated.

### Intra-Dispatch Bypass
When slot 0 and slot 1 dispatch in the same cycle and slot 0 writes a register that slot 1 reads, the dispatch unit forwards slot 0's new physical tag to slot 1's source tag directly. Bypassed sources receive `src_rdy = 0` since the result is not yet in the PRF.

### Store-to-Load Forwarding
When a load executes, the LSQ searches all older STQ entries for address matches. The youngest matching older store with compatible size and valid data forwards directly to the load without a cache access. Unknown-address older stores cause replay.

---

## Known Limitations

| Limitation | Impact | Full Fix |
|---|---|---|
| ROB exec-complete port collision | Simultaneous completions from multiple units to port 1 may drop one | Per-unit ROB writeback ports or completion queue |
| MRET not decoded | Trap return not implemented | Add MRET to decode unit, wire to CSR mret_en |
| No interrupt support | Only synchronous exceptions | Add external interrupt input and mip/mie logic |

---

## Repository Structure

```
rv64im-ooo-processor/
├── README.md
├── rtl/
│   ├── ooo_core.sv          ← Top level
│   ├── fetch_unit.sv
│   ├── branch_predictor.sv
│   ├── decode_unit.sv
│   ├── dispatch_unit.sv
│   ├── free_list.sv
│   ├── rat.sv
│   ├── retire_rat.sv
│   ├── rob.sv
│   ├── issue_queue.sv
│   ├── alu_64.sv
│   ├── branch_unit.sv
│   ├── mul_unit.sv
│   ├── div_unit.sv
│   ├── prf.sv
│   ├── lsq.sv
│   ├── csr.sv
│   └── ld_st_unit.sv
└── tb/
    └── ooo_tb.sv            ← Self-checking testbench
```

---

## How to Simulate

### Vivado XSim

1. Create a new Vivado project targeting any 7-series or UltraScale+ device
2. Add all `.sv` files from `rtl/` as design sources
3. Add `tb/ooo_tb.sv` as a simulation source
4. Set `ooo_tb` as the simulation top
5. Run Behavioral Simulation
6. Expected output: `PASS: 43  FAIL: 0`

### Compilation Order

All modules compile independently. No packages. No `include` directives. Flat packed bus vectors throughout — no cross-file type dependencies.

---

## Tools

| Tool | Version |
|---|---|
| Simulator | Vivado XSim 2025.2 |
| HDL | SystemVerilog (IEEE 1800-2017) |
| Target | Simulation only |
| ISA reference | RISC-V RV64IM Unprivileged Spec v2.2 |
Block Diagram: https://claude.ai/code/artifact/e7e2378c-c100-433e-92ed-1e69324f8cc7
---

## Author

**Joy** — RTL and Verification Engineer  
Portfolio project targeting senior RTL design roles at top-tier semiconductor companies.

Additional portfolio projects:
- 4×4 AXI4 Full-Matrix Crossbar NoC
- 4-core Directory-Based MESI Cache Coherence Controller
- AHB-Lite Slave Verification Environment
- AXI4-Lite Slave with UVM Testbench
