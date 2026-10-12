# Systolic MAC Array Accelerator (AXI4-Lite SoC Subsystem)

[![simulation](https://github.com/tarunpratapsingh17/systolic-mac-array/actions/workflows/sim.yml/badge.svg)](https://github.com/tarunpratapsingh17/systolic-mac-array/actions/workflows/sim.yml)

A **weight-stationary systolic array** of int8 multiply-accumulate units for
neural-network style matrix multiplication, packaged as an SoC subsystem: an
**AXI4-Lite** register slave on the host clock, the array on its own compute
clock, and **asynchronous FIFOs** for the clock-domain crossing. Verified with
**cocotb** unit tests and a **pyuvm** (UVM in Python) environment against a
Python reference model.

```
  aclk domain (host)                              core_clk domain (compute)
 ┌──────────────────┐   command async FIFO   ┌────────────────┐   ┌──────────────────────┐
 │ AXI4-Lite slave  │ ─────────────────────► │ mac_controller │ ─►│ systolic_array N x N │
 │ register map     │ ◄───────────────────── │                │ ◄─│ int8 x int8 → int32  │
 └──────────────────┘   result async FIFO    └────────────────┘   └──────────────────────┘
```

## Highlights

- Parameterized N×N array (default 4×4) of int8×int8→int32 MAC processing elements
- Weight-stationary dataflow: **one result row per cycle**, latency 2N−1 cycles
- AXI4-Lite slave with back-pressure, SLVERR responses and an interrupt line
- Two clock domains joined by Gray-code async FIFOs (from
  [async-fifo](https://github.com/tarunpratapsingh17/async-fifo)) and a reset synchronizer
- Global stall: when the host stops reading, the array pauses instead of dropping results
- **41 cocotb / pyuvm tests pass** across array sizes 2, 3, 4, 8 and three clock ratios
- **Fault injection: 11/11** planted bugs caught by the testbenches
- Verilator `-Wall` lint-clean; synthesizes to 16 DSP48E1, ~176 LUTs, 1,046 FFs (4×4, Yosys)

## How the array works

```
          W[k][j] stays in PE(k,j)            partial sums flow down ↓
                                           0        0        0        0
                                           ↓        ↓        ↓        ↓
 A[m][0] ───────────────────────────► PE(0,0) → PE(0,1) → PE(0,2) → PE(0,3)
 A[m][1] ──[1 cycle]────────────────► PE(1,0) → PE(1,1) → PE(1,2) → PE(1,3)
 A[m][2] ──[2 cycles]───────────────► PE(2,0) → PE(2,1) → PE(2,2) → PE(2,3)
 A[m][3] ──[3 cycles]───────────────► PE(3,0) → PE(3,1) → PE(3,2) → PE(3,3)
           input skew                   ↓        ↓        ↓        ↓
                                     [3 cyc]  [2 cyc]  [1 cyc]     │    output de-skew
                                        ↓        ↓        ↓        ↓
                                     C[m][0]  C[m][1]  C[m][2]  C[m][3]
```

Each PE computes `psum_out = psum_in + a_in × w` and passes the activation to
the right. Because activation `A[m][k]` reaches `PE(k,j)` at the same time as the
partial sum of column `j`, the bottom of column `j` produces
`C[m][j] = Σ_k A[m][k] · W[k][j]`. The input skew lines rows up on the way in
and the de-skew lines columns up on the way out, so a full row of C leaves
together. A new activation row can enter every cycle.

| Module | Purpose |
|--------|---------|
| `rtl/mac_pe.v` | Processing element: weight register, MAC, activation/psum pipeline registers |
| `rtl/systolic_array.v` | N×N PE grid, input skew, output de-skew, valid pipeline, global enable |
| `rtl/mac_controller.v` | Compute-domain sequencer: weight loading, activation issue, back-pressure |
| `rtl/systolic_top.v` | AXI4-Lite slave, register map, CDC FIFOs, reset synchronizer |
| `rtl/async_fifo.v` + helpers | Dual-clock FIFO with Gray-code pointers |
| `rtl/reset_sync.v` | Async-assert / sync-deassert reset into the compute domain |

### Design decisions

- **Weight-stationary** — weights are loaded once and reused for every
  activation row, so streaming M rows costs M + 2N − 1 cycles.
- **Safe weight reload** — the controller only loads a new weight row when no
  activation row is still inside the array, so every result uses one
  consistent weight matrix. It tracks in-flight rows with a counter.
- **Back-pressure instead of data loss** — the whole array has one enable
  (`en = !result_fifo_full`). If the host is slow to read, the array freezes in
  place; when the command FIFO also fills, AXI writes are held off with
  AWREADY/WREADY low.
- **CDC through FIFOs only** — every signal that crosses clock domains goes
  through an async FIFO or the reset synchronizer. STATUS bits are taken from
  the host-side ports of the FIFOs, so no extra synchronizers are needed.

## Register map (AXI4-Lite, 32-bit)

| Offset | Name | Access | Description |
|-------:|------|:------:|-------------|
| 0x00 | `WEIGHT_ROW` | W | Next row of W (rows 0…N−1 in order), int8 element *i* in bits [8i+7:8i] |
| 0x04 | `ACT_ROW` | W | One row of A to multiply, same packing |
| 0x08 | `RESULT` | R | Next int32 of the result; N reads return one row, popped after the N-th (SLVERR if none) |
| 0x0C | `STATUS` | R | [0] cmd_full · [1] result_valid · [2] cmd_almost_full · [15:8] cmd level · [23:16] result rows · [31:24] RESULT lane |
| 0x10 | `ID` | R | `0x534D4143` ("SMAC") |
| 0x14 | `CONFIG` | R | [7:0] N · [15:8] DW · [23:16] ACC |

`irq` is high while a result row is waiting.

## Verification

```
   test ── env ─┬─ agent ─┬─ sequencer ──► driver ──► AXI4-Lite BFM ──► DUT
                │         └─ monitor (passive bus snoop) ──┐
                └─ scoreboard (Python reference model) ◄───┘
```

**cocotb unit tests**
- `test_mac_pe.py` — 2,000 random cycles: weight loads, stalls, extreme
  values, 32-bit wrap-around, checked every cycle
- `test_systolic_array.py` — run at **N = 2, 3, 4, 8**: identity weights,
  back-to-back throughput (64 rows in 71 cycles at N=4: one row per cycle plus
  2N−1 latency), random stalls and input gaps with 6 weight reloads, and
  −128/127 corner values

**pyuvm testbench** (`test_systolic_top.py`) — run at **N = 2 and 4**, each
with the compute clock **faster** (4 ns vs 10 ns), **slower** (23 ns) and
**nearly equal** (10.3 ns) to the AXI clock

| Test | What it does | N = 4, fast core |
|------|--------------|------------------|
| `SmokeTest` | identity weights, results must equal inputs | 8 rows |
| `RandomTest` | 6 random weight sets, 5–40 rows each, random interleaved reads | 141 rows |
| `BackpressureTest` | 96 rows without reading: result FIFO fills (16/16), array stalls, command FIFO fills, then a weight reload with work queued | 96 rows |
| `ErrorResponseTest` | empty RESULT read, writes to read-only/unmapped registers, unmapped read → SLVERR | 10 rows |

The monitor only watches the AXI bus, so the scoreboard checks what actually
happened on the bus rather than what the driver intended.

### Fault injection — 11/11 bugs caught

`scripts/fault_injection.py` plants a bug in a copy of the RTL and re-runs the
tests; each mutation must make a test fail.

| Planted bug | Caught by |
|-------------|-----------|
| PE multiplies unsigned | array tests |
| PE ignores the stall enable | array tests |
| Input skew missing on array row 1 | array tests |
| Output de-skew missing | array tests |
| Valid pipeline one stage short | array tests |
| Weights reloaded while rows are in flight | pyuvm top tests |
| Array not stalled when result FIFO is full | pyuvm top tests |
| Result row popped after every AXI read | pyuvm top tests |
| Weight / activation tag inverted | pyuvm top tests |
| STATUS cmd_full bit stuck at 0 | pyuvm top tests (deadlock → timeout) |
| Empty RESULT read returns OKAY | pyuvm top tests |

## Synthesis

Yosys `synth_xilinx -family xc7`, 4×4 array with both FIFOs:

| Resource | Count |
|----------|------:|
| DSP48E1 | 16 (one MAC per PE) |
| LUTs | ~176 |
| Flip-flops | 1,046 |
| LUT RAM (FIFO storage) | 28 × RAM32M |

## Running it

```bash
pip install -r requirements.txt          # cocotb, pyuvm, pytest
pytest -v tests/test_runner.py           # all configurations (needs Icarus Verilog)
python3 scripts/fault_injection.py       # testbench quality check
WAVES=1 pytest tests/test_runner.py -k "core_fast and 4"   # with waveforms
```

## Limitations / next steps

- The AXI wrapper packs one row per 32-bit word, so it supports N×8 ≤ 32 (N ≤ 4);
  the array itself is tested up to 8×8
- One weight matrix at a time (no double-buffered weights yet)
- Next: double-buffered weights to hide reload time, AXI-Stream data path or DMA,
  output requantization (int32 → int8), tiling for matrices larger than N×N

## License

MIT — see [LICENSE](LICENSE).
