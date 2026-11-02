
# RV64IM 2-Wide Superscalar Out-of-Order Processor

A synthesizable-style **SystemVerilog** implementation of a 2-wide, speculative, out-of-order RISC-V (RV64IM) core with register renaming, a Gshare branch predictor, a reorder buffer, a collapsing issue queue and a load/store queue. Built and verified in **Vivado 2025.2 / XSim** (simulation only).

**Status: 43 / 43 directed tests passing across 15 test programs.**

---

## 1. Why this design exists

An in-order pipeline stalls whenever one instruction waits (a divide, a cache miss, a dependency). An out-of-order core keeps going: it executes any instruction whose operands are ready, yet **commits results in program order**, so software still sees a simple sequential machine. This project builds that machine from first principles, one subsystem at a time.

The core idea has three phases:

1. **In-order front-end** - fetch, predict, decode, rename.
2. **Out-of-order core** - instructions wait in the issue queue and execute when ready.
3. **In-order commit** - the ROB retires instructions strictly in program order, which gives precise exceptions and safe misprediction recovery.

## 2. Architecture at a glance

<img width="3440" height="2800" alt="RV64G OoO Processor-selection" src="https://github.com/user-attachments/assets/94bf6386-59f0-4397-9dce-bcee65d71a71" /># RV64IM 2-Wide Superscalar Out-of-Order Processor

### Key parameters

| Parameter | Value |
|---|---|
| ISA | RV64IM (Unprivileged Spec v2.2), minimal M-mode CSRs |
| Issue / commit width | 2 |
| Physical registers | 96 x 64-bit |
| ROB | 64 entries |
| Issue queue | 16 entries, unified, collapsing |
| Load queue / Store queue | 16 / 16 entries |
| Branch predictor | Gshare: 12-bit GHR, 4096-entry PHT |
| BTB / RAS | 512 / 16 entries |
| MUL | 2 cycles |
| DIV | restoring divider, 32-64 cycles |
| Reset | synchronous, active-low |

## 3. How each part works

**Fetch and branch prediction.** Two instructions are fetched per cycle. Direction comes from Gshare (PHT index = GHR XOR PC bits), so the same branch reached by different paths uses different counters. A redirect is taken only when the PHT predicts *taken* **and** the BTB hits. The RAS predicts function returns.

**Rename.** Architectural registers x0-x31 are mapped to 96 physical registers. This removes false WAR/WAW dependencies, leaving only true RAW ones. The speculative RAT is updated at rename; the **retire RAT** holds the committed mapping. When two instructions are renamed together, slot 1 sees slot 0's new tag through a bypass. A physical register is returned to the free list only when the instruction that replaced its mapping commits. Free-list head pointers are checkpointed, so misprediction recovery of the free list is O(1).

**Reorder buffer.** A 64-entry circular FIFO. Every instruction gets an entry at dispatch, marks itself complete after execution, and retires from the head. Only at commit are the retire RAT and the data cache updated.

**Issue queue.** Collapsing: when an entry issues, younger entries shift up, so index 0 is always the oldest. Each entry compares its source tags with the broadcast result tags (wakeup); ready entries compete and the oldest wins (select). An entry is freed when it issues, whereas its ROB entry lives until commit.

**Load/store queue.** Loads and stores get an LSQ entry in addition to their ROB and IQ entries. Stores keep their data in the store queue and reach the cache only after commit, so wrong-path stores never change memory. A load searches older stores and forwards from the **youngest matching older store**.

**Execution units.** Two ALUs (1 cycle), a branch unit, a pipelined multiplier, an iterative divider and a load/store address unit. ALU opcodes are aligned to `funct3`, which keeps decode simple.

**Recovery.** On a misprediction, everything younger than the branch is flushed and fetch restarts at the correct target. On an exception, the faulting instruction and everything younger are flushed and fetch goes to the trap handler.

## 4. Repository layout

```
rv64im-ooo-processor/
|-- README.md
|-- rtl/
|   |-- ooo_core.sv        top-level integration
|   |-- fetch_unit.sv      PC and I-cache interface
|   |-- branch_predictor.sv  Gshare + BTB + RAS
|   |-- decode_unit.sv
|   |-- dispatch_unit.sv   rename and allocation
|   |-- free_list.sv
|   |-- rat.sv             speculative RAT
|   |-- retire_rat.sv      committed RAT
|   |-- rob.sv
|   |-- issue_queue.sv
|   |-- prf.sv
|   |-- alu_64.sv
|   |-- branch_unit.sv
|   |-- mul_unit.sv
|   |-- div_unit.sv
|   |-- ld_st_unit.sv
|   |-- lsq.sv
|   `-- csr.sv
`-- tb/
    `-- ooo_tb.sv          self-checking testbench
```

## 5. Verification

The testbench `tb/ooo_tb.sv` is self-checking and includes I-cache and D-cache memory models.

* **15 directed test programs**, 43 checks: ALU and word ops, RAW bypass, branch prediction and recovery, JAL/JALR, loads and stores, store-to-load forwarding, MUL, DIV, divide-by-zero, and 2-wide dispatch.
* **13 SVA assertions** on the pipeline's key invariants.
* **5 functional coverage groups.**

## 6. How to simulate

1. Create a new Vivado project (2025.2 recommended).
2. Add everything in `rtl/` as design sources and `tb/ooo_tb.sv` as a simulation source.
3. Set `ooo_tb` as the simulation top.
4. Run **Behavioral Simulation**. The testbench prints a pass/fail summary at the end.

## 7. Design decisions worth knowing

* The collapsing issue queue keeps oldest-first select trivial: lowest index wins.
* Branch redirects need both a taken prediction and a BTB hit.
* Same-cycle dispatch bypass forwards slot 0's new tag to slot 1.
* Store-to-load forwarding picks the youngest older matching store.

## 8. Known limitations

* ROB execute-complete port collisions can drop a completion.
* `MRET` is not decoded.
* No interrupt support.
* Simulation only: no synthesis or timing results are claimed.

## 9. Possible next steps

* Fix the ROB completion-port collision (arbitrate or add write ports).
* Decode `MRET` and add interrupts.
* Add a UVM environment with a commit-trace scoreboard against Spike and constrained-random tests.
* Run synthesis and study the wakeup-select path and routing congestion.

## 10. Tools and references

Vivado XSim 2025.2, SystemVerilog (IEEE 1800-2017), RISC-V RV64IM Unprivileged Spec v2.2.

## Author

**Joy** - RTL design and verification engineer.
