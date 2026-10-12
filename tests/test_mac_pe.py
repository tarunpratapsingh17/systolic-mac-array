"""
cocotb unit test for mac_pe: random weight loads, activations, partial sums and
stall (en = 0) cycles, checked cycle-by-cycle against a Python model.
"""
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge

from ref_model import wrap_signed

DW, ACC = 8, 32


@cocotb.test()
async def pe_random_cycles(dut):
    Clock(dut.clk, 10, unit="ns").start()
    dut.rst_n.value = 0
    dut.en.value = 0
    dut.w_load.value = 0
    dut.w_in.value = 0
    dut.a_in.value = 0
    dut.psum_in.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # model state
    w = a_out = psum_out = 0
    checked = stalls = loads = 0

    for _ in range(2000):
        await FallingEdge(dut.clk)
        en = int(random.random() < 0.8)
        w_load = int(random.random() < 0.1)
        w_in = random.randint(-128, 127)
        a_in = random.choice([-128, 127, random.randint(-128, 127)])
        psum_in = random.choice([0, 2**31 - 1, -2**31, random.randint(-2**31, 2**31 - 1)])

        dut.en.value = en
        dut.w_load.value = w_load
        dut.w_in.value = w_in & 0xFF
        dut.a_in.value = a_in & 0xFF
        dut.psum_in.value = psum_in & 0xFFFFFFFF

        # model: same next-state function as the RTL (uses the OLD weight)
        if en:
            a_out = a_in
            psum_out = wrap_signed(psum_in + a_in * w, ACC)
        if w_load:
            w = w_in
        stalls += not en
        loads += w_load

        await RisingEdge(dut.clk)
        await ReadOnly()
        assert dut.w.value.to_signed() == w, "weight register mismatch"
        assert dut.a_out.value.to_signed() == a_out, "a_out mismatch"
        assert dut.psum_out.value.to_signed() == psum_out, (
            f"psum_out mismatch: got {dut.psum_out.value.to_signed()} expected {psum_out}")
        checked += 1

    dut._log.info(f"checked {checked} cycles ({stalls} stalls, {loads} weight loads)")
