#!/usr/bin/env python3
"""
Fault injection: plant a bug into a copy of the RTL and check that the cocotb /
pyuvm tests catch it. Every mutation must make at least one test fail.

    python3 scripts/fault_injection.py

Uses one array configuration (N=4) and one top-level configuration (N=4, core
clock faster than AXI clock) to keep the run short.
"""
import io
import os
import shutil
import sys
import tempfile
from contextlib import redirect_stderr, redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests"))
from test_runner import ARRAY_SRCS, TOP_SRCS, run  # noqa: E402

# (description, file, original text, mutated text)
MUTATIONS = [
    ("PE multiplies unsigned",
     "mac_pe.v", "wire signed [2*DW-1:0] prod = a_in * w;",
     "wire signed [2*DW-1:0] prod = $unsigned(a_in) * $unsigned(w);"),
    ("PE ignores the stall enable",
     "mac_pe.v", "            if (en) begin\n                a_out", "            if (1'b1) begin\n                a_out"),
    ("input skew missing on array row 1",
     "systolic_array.v", "assign a_skew[k*DW +: DW] = sr[k*DW-1 -: DW];",
     "assign a_skew[k*DW +: DW] = (k == 1) ? a_row[k*DW +: DW] : sr[k*DW-1 -: DW];"),
    ("output de-skew missing",
     "systolic_array.v", "assign c_row[j*ACC +: ACC] = sr[D*ACC-1 -: ACC];",
     "assign c_row[j*ACC +: ACC] = col_out;"),
    ("valid pipeline one stage short",
     "systolic_array.v", "assign c_valid = vpipe[2*N-2];", "assign c_valid = vpipe[2*N-3];"),
    ("weights reloaded while rows are in flight",
     "mac_controller.v", "wire take_weight = en && !cmd_empty &&  head_is_weight && array_idle;",
     "wire take_weight = en && !cmd_empty &&  head_is_weight;"),
    ("array not stalled when result FIFO is full",
     "mac_controller.v", "assign en = ~res_full;", "assign en = 1'b1;"),
    ("result row popped after every AXI read",
     "systolic_top.v", "assign res_rd  = rd_result && last_lane;", "assign res_rd  = rd_result;"),
    ("weight/activation tag inverted",
     "systolic_top.v", "assign cmd_wdata      = {(waddr == ADDR_WEIGHT),",
     "assign cmd_wdata      = {(waddr == ADDR_ACT),"),
    ("STATUS cmd_full bit stuck at 0",
     "systolic_top.v", "5'd0, cmd_almost_full, ~res_empty, cmd_full};",
     "5'd0, cmd_almost_full, ~res_empty, 1'b0};"),
    ("empty RESULT read returns OKAY",
     "systolic_top.v", "                        s_axil_rresp <= RESP_SLVERR;\n                    end else begin",
     "                        s_axil_rresp <= RESP_OKAY;\n                    end else begin"),
]

ARRAY_RUN = dict(toplevel="systolic_array", srcs=ARRAY_SRCS, module="test_systolic_array",
                 name="fi_array", parameters={"N": 4}, env={"SYS_N": "4"})
TOP_RUN = dict(toplevel="systolic_top", srcs=TOP_SRCS, module="test_systolic_top",
               name="fi_top", parameters={"N": 4},
               env={"SYS_N": "4", "ACLK_NS": "10", "CORE_NS": "4"})


def caught(rtl_dir, build_root):
    for cfg, label in ((ARRAY_RUN, "array tests"), (TOP_RUN, "pyuvm top tests")):
        try:
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                run(rtl_dir=rtl_dir, build_root=build_root, quiet=True, **cfg)
        except AssertionError:     # a test failed (wrong data, protocol error or timeout)
            return label
    return None


def main():
    killed = 0
    with tempfile.TemporaryDirectory() as tmp:
        if caught(os.path.join(ROOT, "rtl"), tmp):
            sys.exit("baseline RTL fails - fix that first")
        for desc, fname, orig, mut in MUTATIONS:
            mdir = os.path.join(tmp, "rtl")
            shutil.rmtree(mdir, ignore_errors=True)
            shutil.copytree(os.path.join(ROOT, "rtl"), mdir)
            path = os.path.join(mdir, fname)
            src = open(path).read()
            if orig not in src:
                print(f"SKIP      {desc}: pattern not found")
                continue
            open(path, "w").write(src.replace(orig, mut, 1))
            by = caught(mdir, tmp)
            if by:
                killed += 1
                print(f"KILLED    {desc:<44} caught by {by}")
            else:
                print(f"SURVIVED  {desc:<44} no test noticed")
    print("-" * 78)
    print(f"{killed}/{len(MUTATIONS)} injected bugs caught by the testbenches")
    sys.exit(0 if killed == len(MUTATIONS) else 1)


if __name__ == "__main__":
    main()
