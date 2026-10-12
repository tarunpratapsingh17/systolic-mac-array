"""
pyuvm testbench for systolic_top (AXI4-Lite accelerator with two clock domains).

    test ── env ─┬─ agent ─┬─ sequencer ──► driver ──► AxiLiteBfm ──► DUT
                 │         └─ monitor (passive AXI bus snoop) ──┐
                 └─ scoreboard ◄────────── analysis port ───────┘

* The driver turns high-level items (load weights / send an activation row /
  read results / drain) into AXI4-Lite transfers, polling STATUS so it never
  deadlocks on a full command FIFO.
* The monitor watches the AXI bus only, so the scoreboard checks what really
  happened on the bus, independently of what the driver intended.
* The scoreboard runs the Python reference model (A @ W in int32) and compares
  every result row, in order.

Environment variables (set by tests/test_runner.py):
    SYS_N      array size N            (default 4)
    ACLK_NS    AXI clock period in ns  (default 10)
    CORE_NS    core clock period in ns (default 4)
"""
import os
import random

import cocotb
import pyuvm
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge
from pyuvm import (ConfigDB, uvm_agent, uvm_analysis_port, uvm_driver, uvm_env, uvm_monitor,
                   uvm_sequence, uvm_sequence_item, uvm_sequencer, uvm_subscriber, uvm_test)

from ref_model import matmul_row, pack, rand_int8_matrix, rand_int8_row, unpack

N = int(os.environ.get("SYS_N", "4"))
ACLK_NS = float(os.environ.get("ACLK_NS", "10"))
CORE_NS = float(os.environ.get("CORE_NS", "4"))
DW, ACC, FIFO_DEPTH = 8, 32, 16

REG_WEIGHT, REG_ACT, REG_RESULT, REG_STATUS, REG_ID, REG_CONFIG = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14
OKAY, SLVERR = 0, 2
ID_VALUE = 0x534D4143


def status_fields(st):
    return {"cmd_full": st & 1, "result_valid": (st >> 1) & 1, "cmd_almost_full": (st >> 2) & 1,
            "cmd_level": (st >> 8) & 0xFF, "res_rows": (st >> 16) & 0xFF, "lane": (st >> 24) & 0xFF}


# =============================================================================
# Bus functional model
# =============================================================================
class AxiLiteBfm:
    """Drives AXI4-Lite transfers. Signals change only on the falling edge of
    aclk; handshakes complete on the rising edge."""

    def __init__(self, dut):
        self.dut = dut
        self.clk = dut.aclk

    async def reset(self):
        d = self.dut
        for s in (d.s_axil_awvalid, d.s_axil_wvalid, d.s_axil_arvalid, d.s_axil_awaddr,
                  d.s_axil_wdata, d.s_axil_araddr):
            s.value = 0
        d.s_axil_wstrb.value = 0xF
        d.s_axil_bready.value = 1
        d.s_axil_rready.value = 1
        d.aresetn.value = 0
        await ClockCycles(self.clk, 5)
        await FallingEdge(self.clk)
        d.aresetn.value = 1
        await ClockCycles(self.clk, 5)       # let the core-domain reset sync release

    async def write(self, addr, data):
        d = self.dut
        await FallingEdge(self.clk)
        d.s_axil_awaddr.value = addr
        d.s_axil_wdata.value = data & 0xFFFFFFFF
        d.s_axil_awvalid.value = 1
        d.s_axil_wvalid.value = 1
        while True:                          # wait for AWREADY/WREADY
            await ReadOnly()
            if int(d.s_axil_awready.value) and int(d.s_axil_wready.value):
                break
            await FallingEdge(self.clk)
        await RisingEdge(self.clk)           # address + data handshake
        await FallingEdge(self.clk)
        d.s_axil_awvalid.value = 0
        d.s_axil_wvalid.value = 0
        assert int(d.s_axil_bvalid.value), "BVALID missing after write"
        return int(d.s_axil_bresp.value)

    async def read(self, addr):
        d = self.dut
        await FallingEdge(self.clk)
        d.s_axil_araddr.value = addr
        d.s_axil_arvalid.value = 1
        while True:
            await ReadOnly()
            if int(d.s_axil_arready.value):
                break
            await FallingEdge(self.clk)
        await RisingEdge(self.clk)           # address handshake, RVALID rises
        await FallingEdge(self.clk)
        d.s_axil_arvalid.value = 0
        assert int(d.s_axil_rvalid.value), "RVALID missing after read"
        return int(d.s_axil_rdata.value), int(d.s_axil_rresp.value)


# =============================================================================
# Transactions
# =============================================================================
class MacItem(uvm_sequence_item):
    """kind: WEIGHTS (data = N x N matrix), ACT (data = row), READ_ROW, DRAIN, ERRORS"""

    def __init__(self, name="MacItem", kind="ACT", data=None):
        super().__init__(name)
        self.kind = kind
        self.data = data


class BusTxn:
    def __init__(self, op, addr, data, resp):
        self.op, self.addr, self.data, self.resp = op, addr, data, resp


# =============================================================================
# Sequences
# =============================================================================
class BaseSeq(uvm_sequence):
    async def send(self, kind, data=None):
        item = MacItem(kind=kind, data=data)
        await self.start_item(item)
        await self.finish_item(item)


class SmokeSeq(BaseSeq):
    """Identity weights: results must equal the activations."""
    async def body(self):
        eye = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
        await self.send("WEIGHTS", eye)
        for _ in range(8):
            await self.send("ACT", rand_int8_row(N))
        await self.send("DRAIN")


class RandomSeq(BaseSeq):
    """Several random weight sets, random batch sizes, reads interleaved at random."""
    async def body(self):
        for _ in range(6):
            await self.send("WEIGHTS", rand_int8_matrix(N))
            for _ in range(random.randint(5, 40)):
                await self.send("ACT", rand_int8_row(N))
                if random.random() < 0.35:
                    await self.send("READ_ROW")
        await self.send("DRAIN")


class BackpressureSeq(BaseSeq):
    """Write far more rows than both FIFOs hold without reading: the result FIFO
    fills, the array stalls, the command FIFO fills and the driver is forced to
    read results to make progress. Then a weight reload while rows are queued."""
    async def body(self):
        await self.send("WEIGHTS", rand_int8_matrix(N))
        for _ in range(4 * FIFO_DEPTH):
            await self.send("ACT", rand_int8_row(N))
        await self.send("WEIGHTS", rand_int8_matrix(N))       # reload with work queued
        for _ in range(2 * FIFO_DEPTH):
            await self.send("ACT", rand_int8_row(N))
        await self.send("DRAIN")


class ErrorSeq(BaseSeq):
    """Protocol corner cases: empty RESULT read, writes to read-only/unmapped
    registers, unmapped reads - all must return SLVERR and change nothing."""
    async def body(self):
        await self.send("ERRORS")
        await self.send("WEIGHTS", rand_int8_matrix(N))
        for _ in range(10):
            await self.send("ACT", rand_int8_row(N))
        await self.send("DRAIN")
        await self.send("ERRORS")


# =============================================================================
# Agent components
# =============================================================================
class Driver(uvm_driver):
    def build_phase(self):
        self.bfm = ConfigDB().get(self, "", "BFM")
        self.pending = 0                     # activation rows written, results not yet read
        self.cov = ConfigDB().get(self, "", "COV")

    async def status(self):
        data, resp = await self.bfm.read(REG_STATUS)
        assert resp == OKAY
        st = status_fields(data)
        self.cov["max_res_rows"] = max(self.cov["max_res_rows"], st["res_rows"])
        self.cov["max_cmd_level"] = max(self.cov["max_cmd_level"], st["cmd_level"])
        self.cov["cmd_full_seen"] += st["cmd_full"]
        return st

    async def read_row(self):
        for lane in range(N):
            _, resp = await self.bfm.read(REG_RESULT)
            assert resp == OKAY, f"RESULT lane {lane} returned SLVERR with a row waiting"
        self.pending -= 1

    async def wait_and_read_row(self):
        while not (await self.status())["result_valid"]:
            await ClockCycles(self.bfm.clk, 2)
        await self.read_row()

    async def run_phase(self):
        while True:
            item = await self.seq_item_port.get_next_item()
            await self.execute(item)
            self.seq_item_port.item_done()

    async def execute(self, item):
        if item.kind == "WEIGHTS":
            self.cov["weight_loads"] += 1
            for row in item.data:
                await self.wait_cmd_space()
                assert await self.bfm.write(REG_WEIGHT, pack(row, DW)) == OKAY
        elif item.kind == "ACT":
            await self.wait_cmd_space()
            assert await self.bfm.write(REG_ACT, pack(item.data, DW)) == OKAY
            self.pending += 1
        elif item.kind == "READ_ROW":
            if self.pending and (await self.status())["result_valid"]:
                await self.read_row()
        elif item.kind == "DRAIN":
            while self.pending:
                await self.wait_and_read_row()
            st = await self.status()
            assert not st["result_valid"] and st["lane"] == 0, f"leftover state after drain: {st}"
        elif item.kind == "ERRORS":
            assert self.pending == 0
            data, resp = await self.bfm.read(REG_RESULT)
            assert resp == SLVERR and data == 0, "empty RESULT read must return SLVERR"
            for addr in (REG_RESULT, REG_STATUS, REG_ID, REG_CONFIG, 0x40):
                assert await self.bfm.write(addr, 0xDEADBEEF) == SLVERR, f"write to {addr:#x} not rejected"
            _, resp = await self.bfm.read(0x40)
            assert resp == SLVERR, "unmapped read must return SLVERR"
            self.cov["error_checks"] += 1

    async def wait_cmd_space(self):
        """Never write into a full command FIFO: if it is full the array is
        probably stalled on a full result FIFO, so read a result to unblock it."""
        while (await self.status())["cmd_full"]:
            self.cov["forced_drains"] += 1
            if self.pending:
                await self.wait_and_read_row()
            else:
                await ClockCycles(self.bfm.clk, 4)


class Monitor(uvm_monitor):
    """Passive: reconstructs completed AXI transfers from the bus signals."""

    def build_phase(self):
        self.ap = uvm_analysis_port("ap", self)
        self.dut = cocotb.top

    async def run_phase(self):
        d = self.dut
        pending_read = None
        while True:
            await FallingEdge(d.aclk)
            await ReadOnly()
            if not int(d.aresetn.value):
                continue
            # these values are stable until the next rising edge, where the handshakes happen
            if int(d.s_axil_awvalid.value) and int(d.s_axil_awready.value):
                write = (int(d.s_axil_awaddr.value) & ~3, int(d.s_axil_wdata.value))
            else:
                write = None
            if int(d.s_axil_bvalid.value) and int(d.s_axil_bready.value):
                addr, data = self.last_write
                self.ap.write(BusTxn("W", addr, data, int(d.s_axil_bresp.value)))
            if write:
                self.last_write = write
            if int(d.s_axil_rvalid.value) and int(d.s_axil_rready.value):
                self.ap.write(BusTxn("R", pending_read, int(d.s_axil_rdata.value),
                                     int(d.s_axil_rresp.value)))
            if int(d.s_axil_arvalid.value) and int(d.s_axil_arready.value):
                pending_read = int(d.s_axil_araddr.value) & ~3


class Scoreboard(uvm_subscriber):
    """Reference model: keeps the current weight matrix as the bus writes it and
    predicts one result row per activation row."""

    def build_phase(self):
        self.weights = [[0] * N for _ in range(N)]
        self.wrow = 0
        self.expected = []          # queue of predicted rows
        self.lanes = []             # int32 values of the row currently being read
        self.rows_checked = 0
        self.errors = 0

    def error(self, msg):
        self.errors += 1
        self.logger.error(msg)

    def write(self, t):
        if t.op == "W":
            if t.addr == REG_WEIGHT and t.resp == OKAY:
                self.weights[self.wrow] = unpack(t.data, DW, N)
                self.wrow = (self.wrow + 1) % N
            elif t.addr == REG_ACT and t.resp == OKAY:
                self.expected.append(matmul_row(unpack(t.data, DW, N), self.weights, ACC))
            elif t.addr in (REG_WEIGHT, REG_ACT):
                self.error(f"data write to {t.addr:#x} got response {t.resp}")
        else:
            if t.addr == REG_RESULT and t.resp == OKAY:
                v = t.data - (1 << 32) if t.data >> 31 else t.data
                self.lanes.append(v)
                if len(self.lanes) == N:
                    if not self.expected:
                        self.error(f"unexpected result row {self.lanes}")
                    else:
                        exp = self.expected.pop(0)
                        if exp != self.lanes:
                            self.error(f"result row {self.rows_checked}: got {self.lanes}, expected {exp}")
                    self.rows_checked += 1
                    self.lanes = []
            elif t.addr == REG_ID and t.data != ID_VALUE:
                self.error(f"ID register reads {t.data:#x}")
            elif t.addr == REG_CONFIG and (t.data & 0xFF, (t.data >> 8) & 0xFF, (t.data >> 16) & 0xFF) != (N, DW, ACC):
                self.error(f"CONFIG register reads {t.data:#x}")

    def check_phase(self):
        if self.expected:
            self.error(f"{len(self.expected)} result rows were never read back")
        if self.rows_checked == 0:
            self.error("no result rows checked")
        assert self.errors == 0, f"scoreboard found {self.errors} errors"

    def report_phase(self):
        self.logger.info(f"scoreboard: {self.rows_checked} result rows matched the reference model")


class Agent(uvm_agent):
    def build_phase(self):
        self.seqr = uvm_sequencer("seqr", self)
        self.driver = Driver("driver", self)
        self.monitor = Monitor("monitor", self)

    def connect_phase(self):
        self.driver.seq_item_port.connect(self.seqr.seq_item_export)


class Env(uvm_env):
    def build_phase(self):
        self.agent = Agent("agent", self)
        self.scoreboard = Scoreboard("scoreboard", self)

    def connect_phase(self):
        self.agent.monitor.ap.connect(self.scoreboard.analysis_export)


# =============================================================================
# Tests
# =============================================================================
class BaseTest(uvm_test):
    SEQ = SmokeSeq

    def build_phase(self):
        self.dut = cocotb.top
        self.bfm = AxiLiteBfm(self.dut)
        self.cov = {"weight_loads": 0, "max_res_rows": 0, "max_cmd_level": 0,
                    "cmd_full_seen": 0, "forced_drains": 0, "error_checks": 0}
        ConfigDB().set(None, "*", "BFM", self.bfm)
        ConfigDB().set(None, "*", "COV", self.cov)
        self.env = Env("env", self)

    async def run_phase(self):
        self.raise_objection()
        Clock(self.dut.aclk, round(ACLK_NS * 1000), unit="ps").start()
        Clock(self.dut.core_clk, round(CORE_NS * 1000), unit="ps").start()
        await self.bfm.reset()

        ident, resp = await self.bfm.read(REG_ID)
        assert resp == OKAY and ident == ID_VALUE, f"bad ID {ident:#x}"
        cfg, _ = await self.bfm.read(REG_CONFIG)
        assert cfg & 0xFF == N, f"CONFIG says N={cfg & 0xFF}, test expects {N}"

        await self.SEQ("seq").start(self.env.agent.seqr)
        await ClockCycles(self.dut.aclk, 20)
        self.drop_objection()

    def check_phase(self):
        self.check_coverage(self.cov)

    def check_coverage(self, cov):
        pass

    def report_phase(self):
        self.logger.info(f"N={N} aclk={ACLK_NS}ns core_clk={CORE_NS}ns coverage: {self.cov}")


@pyuvm.test(timeout_time=3, timeout_unit="ms")
class SmokeTest(BaseTest):
    SEQ = SmokeSeq


@pyuvm.test(timeout_time=3, timeout_unit="ms")
class RandomTest(BaseTest):
    SEQ = RandomSeq

    def check_coverage(self, cov):
        assert cov["weight_loads"] >= 6


@pyuvm.test(timeout_time=3, timeout_unit="ms")
class BackpressureTest(BaseTest):
    SEQ = BackpressureSeq

    def check_coverage(self, cov):
        # the result FIFO must have filled and the command FIFO must have hit full
        assert cov["cmd_full_seen"] > 0, "command FIFO never reported full"
        assert cov["max_res_rows"] >= FIFO_DEPTH - 2, f"result FIFO only reached {cov['max_res_rows']}"


@pyuvm.test(timeout_time=3, timeout_unit="ms")
class ErrorResponseTest(BaseTest):
    SEQ = ErrorSeq

    def check_coverage(self, cov):
        assert cov["error_checks"] == 2
