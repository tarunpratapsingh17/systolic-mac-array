"""
pytest entry point: builds the RTL with Icarus Verilog through the cocotb
runner and runs every cocotb / pyuvm test in several configurations.

    pytest -v tests/test_runner.py
"""
import os
from pathlib import Path

import pytest
from cocotb_tools.runner import get_results, get_runner

ROOT = Path(__file__).resolve().parent.parent
RTL = ROOT / "rtl"
TESTS = ROOT / "tests"
SIM = os.environ.get("SIM", "icarus")

FIFO_SRCS = ["cdc_sync_2ff.v", "fifo_dpram.v", "fifo_wr_ctrl.v", "fifo_rd_ctrl.v", "async_fifo.v"]
ARRAY_SRCS = ["mac_pe.v", "systolic_array.v"]
TOP_SRCS = FIFO_SRCS + ARRAY_SRCS + ["mac_controller.v", "reset_sync.v", "systolic_top.v"]


def run(toplevel, srcs, module, name, parameters=None, env=None, rtl_dir=RTL, build_root=None,
        quiet=False):
    build_dir = Path(build_root or ROOT / "sim_build") / name
    build_dir.mkdir(parents=True, exist_ok=True)
    log = {"log_file": build_dir / "sim.log"} if quiet else {}
    runner = get_runner(SIM)
    runner.build(
        sources=[Path(rtl_dir) / s for s in srcs],
        hdl_toplevel=toplevel,
        parameters=parameters or {},
        build_dir=build_dir,
        always=True,
        timescale=("1ns", "1ps"),
        waves=bool(os.environ.get("WAVES")),
        **log,
    )
    results = runner.test(
        hdl_toplevel=toplevel,
        test_module=module,
        build_dir=build_dir,
        test_dir=build_dir,
        extra_env={"PYTHONPATH": str(TESTS), **(env or {})},
        waves=bool(os.environ.get("WAVES")),
        seed=int(os.environ.get("SEED", "12345")),
        **log,
    )
    n_tests, n_failed = get_results(results)
    assert n_tests > 0, "no tests ran"
    assert n_failed == 0, f"{n_failed} of {n_tests} tests failed"


def test_mac_pe():
    run("mac_pe", ["mac_pe.v"], "test_mac_pe", "mac_pe")


@pytest.mark.parametrize("n", [2, 3, 4, 8])
def test_systolic_array(n):
    run("systolic_array", ARRAY_SRCS, "test_systolic_array", f"array_n{n}",
        parameters={"N": n}, env={"SYS_N": str(n)})


@pytest.mark.parametrize("n", [2, 4])
@pytest.mark.parametrize("aclk_ns,core_ns", [(10, 4), (10, 23), (10, 10.3)],
                         ids=["core_fast", "core_slow", "near_equal"])
def test_systolic_top(n, aclk_ns, core_ns):
    run("systolic_top", TOP_SRCS, "test_systolic_top", f"top_n{n}_{aclk_ns}_{core_ns}",
        parameters={"N": n},
        env={"SYS_N": str(n), "ACLK_NS": str(aclk_ns), "CORE_NS": str(core_ns)})
