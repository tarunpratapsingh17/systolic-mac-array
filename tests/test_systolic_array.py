"""
cocotb tests for systolic_array driven directly (no AXI): the testbench plays
the controller. Random int8 matrices, random stalls (en = 0) and gaps in the
input stream, several weight reloads; every output row is compared with
A @ W from the Python model, in order.

The array size comes from the SYS_N environment variable (set by the runner).
"""
import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge

from ref_model import matmul_row, pack, rand_int8_matrix, rand_int8_row, unpack

N = int(os.environ.get("SYS_N", "4"))
DW, ACC = 8, 32


async def reset(dut):
    Clock(dut.clk, 10, unit="ns").start()
    dut.rst_n.value = 0
    dut.en.value = 0
    dut.w_load_row.value = 0
    dut.w_row.value = 0
    dut.a_valid.value = 0
    dut.a_row.value = 0
    for _ in range(3):
        await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1


async def load_weights(dut, weights):
    for k in range(N):
        await FallingEdge(dut.clk)
        dut.en.value = 0
        dut.a_valid.value = 0
        dut.w_load_row.value = 1 << k
        dut.w_row.value = pack(weights[k], DW)
    await FallingEdge(dut.clk)
    dut.w_load_row.value = 0


async def stream(dut, weights, rows, p_stall, p_gap):
    """Feed `rows`, collect results; inputs/outputs only move on cycles with en = 1."""
    expected = [matmul_row(r, weights, ACC) for r in rows]
    got = []
    idx = 0
    cycles = 0
    while len(got) < len(rows):
        await FallingEdge(dut.clk)
        cycles += 1
        en = int(random.random() >= p_stall)
        # an output row present now leaves the array on the next edge if en = 1
        if en and int(dut.c_valid.value):
            got.append(unpack(int(dut.c_row.value), ACC, N))
        send = en and idx < len(rows) and random.random() >= p_gap
        dut.en.value = en
        dut.a_valid.value = int(send)
        dut.a_row.value = pack(rows[idx], DW) if send else 0
        if send:
            idx += 1
        assert cycles < 50 * (len(rows) + 4 * N), "timeout waiting for results"

    for m, (g, e) in enumerate(zip(got, expected)):
        assert g == e, f"row {m}: got {g}, expected {e}\nA row = {rows[m]}\nW = {weights}"
    return cycles


@cocotb.test()
async def identity_weights(dut):
    """W = I: every output row must equal its input row."""
    await reset(dut)
    eye = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    await load_weights(dut, eye)
    rows = [rand_int8_row(N) for _ in range(20)]
    await stream(dut, eye, rows, p_stall=0.0, p_gap=0.0)


@cocotb.test()
async def back_to_back_throughput(dut):
    """No stalls or gaps: M rows must finish in M + (2N-1) cycles (1 row per cycle)."""
    await reset(dut)
    w = rand_int8_matrix(N)
    await load_weights(dut, w)
    m = 64
    cycles = await stream(dut, w, [rand_int8_row(N) for _ in range(m)], 0.0, 0.0)
    dut._log.info(f"{m} rows in {cycles} cycles (latency {2 * N - 1})")
    assert cycles <= m + 2 * N, f"throughput too low: {cycles} cycles for {m} rows"


@cocotb.test()
async def random_stalls_and_reloads(dut):
    """Random matrices, random en stalls and input gaps, 6 weight reloads."""
    await reset(dut)
    total = 0
    for _ in range(6):
        w = rand_int8_matrix(N)
        await load_weights(dut, w)
        rows = [rand_int8_row(N) for _ in range(random.randint(10, 60))]
        await stream(dut, w, rows, p_stall=0.25, p_gap=0.3)
        total += len(rows)
    dut._log.info(f"{total} rows checked across 6 weight sets")


@cocotb.test()
async def extreme_values(dut):
    """All -128 / 127 corners: largest-magnitude products and sums."""
    await reset(dut)
    for wv in (-128, 127):
        w = [[wv] * N for _ in range(N)]
        await load_weights(dut, w)
        rows = [[-128] * N, [127] * N, [-128, 127] * (N // 2) + [-128] * (N % 2)]
        await stream(dut, w, rows, 0.1, 0.1)
