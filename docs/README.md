# TL-SRAM Main Memory Model

## 1. Overview
This IP provides a synthesizable, TileLink-UH compliant memory controller designed to act as a **Main Memory Substitute** for RISC-V SoC-Flow verification. It supports burst transactions (cache line fills/evictions), configurable latencies, and error injection, satisfying the verification needs for L3 Cache and Coherence testing without the complexity of a DDR PHY.

## 2. Project Specifications
| Category | Spec Requirement | Implementation |
| :--- | :--- | :--- |
| **Protocol** | TileLink UL/UH | Full Support (Get, PutFull, PutPartial) |
| **Bursts** | Cache Line (64B) Support | Supported via `a_size` decoding |
| **Latency** | Configurable Read/Write | Parameters `READ_LATENCY` / `WRITE_LATENCY` |
| **Backpressure** | Realistic Handshakes | Implemented via FIFO Full / Busy States |
| **Ordering** | Deterministic per ID | Unified FIFO ensures strict ordering |
| **Error Handling** | Invalid Opcodes/OOB | Returns `d_corrupt` high |
| **Reset** | Zero Initialization | Simulation: `initial` loop. Synth: none. |

## 3. Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `MEM_SIZE_BYTES` | 64MB | Total memory size (Synthesis impact). |
| `DATA_WIDTH` | 64 | Bus width (Fixed). |
| `QUEUE_DEPTH` | 4 | Number of outstanding requests buffered. |
| `READ_LATENCY` | 1 | Cycles from Req acceptance to First Data. |
| `WRITE_LATENCY` | 1 | Cycles from Last Data beat to Ack. |
| `INIT_FILE` | "" | Path to `.hex` file for firmware loading. |

## 4. Integration Guide

### Instantiation Template
```verilog
tl_sram_ctrl #(
    .READ_LATENCY(4),
    .WRITE_LATENCY(2),
    .INIT_FILE("firmware.hex")
) u_main_mem (
    .clk_i      (clk),
    .rst_ni     (rst_n),
    // TL-A
    .tl_a_valid (mem_a_valid),
    .tl_a_ready (mem_a_ready),
    .tl_a_opcode(mem_a_opcode),
    .tl_a_size  (mem_a_size),
    // ...
    // TL-D
    .tl_d_valid (mem_d_valid),
    .tl_d_ready (mem_d_ready),
    // ...
);
