"""PoC-1: Material + Welding + Inspection 통합 데모.

실행 방법:
    # Isaac Sim 없이 (로직 검증용 — 어디서든 실행 가능)
    python scripts/run_poc1.py --mode discrete

    # AGX Thor의 isaac_sim conda 환경에서 (Isaac Sim 5.x 통합)
    conda activate isaac_sim
    python scripts/run_poc1.py --mode isaac --headless

    # 설정 파일 지정
    python scripts/run_poc1.py --config fdw_sim/config/poc1.yaml --mode discrete
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict

# 프로젝트 루트를 import path에 추가
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from fdw_sim.cells.base.cell_base import CellConfig
from fdw_sim.cells.inspection.inspection_cell import InspectionCell
from fdw_sim.cells.material.material_cell import MaterialCell
from fdw_sim.cells.welding.welding_cell import WeldingCell
from fdw_sim.messaging.bus import InMemoryBus
from fdw_sim.messaging.schemas import CellType, JobSpec
from fdw_sim.orchestrator.orchestrator import OrchestratorConfig
from fdw_sim.simulation.manager import SimulationConfig, SimulationManager
from fdw_sim.utils.logging_setup import setup_logging


def load_config(path: Path) -> Dict[str, Any]:
    """YAML 우선, 실패 시 fallback 기본 dict 사용."""
    try:
        import yaml  # type: ignore
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[WARN] yaml load failed ({e}); using built-in defaults")
        return _builtin_defaults()


def _builtin_defaults() -> Dict[str, Any]:
    """yaml 모듈이 없는 환경 대비 기본 설정."""
    return {
        "simulation": {
            "mode": "discrete",
            "physics_hz": 60.0,
            "max_sim_time_sec": 600.0,
            "realtime": False,
            "headless": True,
            "log_dir": "./fdw_sim/logs",
            "run_name": None,
        },
        "orchestrator": {
            "decision_period_sec": 0.2,
            "material_cell_id": "MATERIAL_CELL_01",
            "process_to_cell": {
                "welding": "WELDING_CELL_01",
                "inspection": "INSPECTION_CELL_01",
            },
        },
        "cells": {
            "material": {"cell_id": "MATERIAL_CELL_01", "location": [0.0, 0.0],
                         "num_amrs": 2, "input_capacity": 8, "output_capacity": 8},
            "welding": {"cell_id": "WELDING_CELL_01", "location": [5.0, 0.0],
                        "default_cycle_time": 18.0,
                        "input_capacity": 1, "output_capacity": 1},
            "inspection": {"cell_id": "INSPECTION_CELL_01", "location": [10.0, 0.0],
                           "default_cycle_time": 6.0,
                           "input_capacity": 1, "output_capacity": 1},
        },
        "jobs": [
            {"job_id": "JOB_00001", "part_id": "PART_A_001",
             "part_type": "tubular_frame_A", "priority": "normal",
             "process_route": ["welding", "inspection"],
             "recipe_overrides": {"welding": {"speed": 0.10, "power": 1500.0,
                                              "path_length_mm": 220.0}}},
            {"job_id": "JOB_00002", "part_id": "PART_A_002",
             "part_type": "tubular_frame_A", "priority": "normal",
             "process_route": ["welding", "inspection"],
             "recipe_overrides": {"welding": {"speed": 0.08, "power": 1700.0,
                                              "path_length_mm": 250.0}}},
            {"job_id": "JOB_00003", "part_id": "PART_B_001",
             "part_type": "tubular_frame_B", "priority": "high",
             "process_route": ["welding", "inspection"],
             "recipe_overrides": {"welding": {"speed": 0.09, "power": 220.0,
                                              "path_length_mm": 180.0}}},
        ],
    }


def build_simulation(cfg: Dict[str, Any], mode_override: str = None,
                     headless_override: bool = None) -> SimulationManager:
    sim_cfg_d = dict(cfg["simulation"])
    if mode_override:
        sim_cfg_d["mode"] = mode_override
    if headless_override is not None:
        sim_cfg_d["headless"] = headless_override

    sim_cfg = SimulationConfig(
        mode=sim_cfg_d["mode"],
        physics_hz=float(sim_cfg_d["physics_hz"]),
        max_sim_time_sec=float(sim_cfg_d["max_sim_time_sec"]),
        realtime=bool(sim_cfg_d["realtime"]),
        headless=bool(sim_cfg_d["headless"]),
        log_dir=Path(sim_cfg_d["log_dir"]),
        run_name=sim_cfg_d.get("run_name"),
    )

    orch_d = cfg["orchestrator"]
    orch_cfg = OrchestratorConfig(
        decision_period_sec=float(orch_d["decision_period_sec"]),
        material_cell_id=orch_d["material_cell_id"],
        process_to_cell=dict(orch_d["process_to_cell"]),
    )

    bus = InMemoryBus()
    sim = SimulationManager(sim_cfg, bus=bus, orchestrator_config=orch_cfg)

    # ---- 셀 인스턴스 생성 ----
    cells_d = cfg["cells"]

    mat_d = cells_d["material"]
    material = MaterialCell(
        config=CellConfig(
            cell_id=mat_d["cell_id"],
            cell_type=CellType.MATERIAL,
            input_capacity=int(mat_d["input_capacity"]),
            output_capacity=int(mat_d["output_capacity"]),
        ),
        bus=bus,
        num_amrs=int(mat_d["num_amrs"]),
    )

    weld_d = cells_d["welding"]
    welding = WeldingCell(
        config=CellConfig(
            cell_id=weld_d["cell_id"],
            cell_type=CellType.WELDING,
            input_capacity=int(weld_d["input_capacity"]),
            output_capacity=int(weld_d["output_capacity"]),
        ),
        bus=bus,
        default_cycle_time=float(weld_d["default_cycle_time"]),
    )

    insp_d = cells_d["inspection"]
    inspection = InspectionCell(
        config=CellConfig(
            cell_id=insp_d["cell_id"],
            cell_type=CellType.INSPECTION,
            input_capacity=int(insp_d["input_capacity"]),
            output_capacity=int(insp_d["output_capacity"]),
        ),
        bus=bus,
        default_cycle_time=float(insp_d["default_cycle_time"]),
    )

    # ---- 등록 ----
    sim.register_material_cell(material, location=tuple(mat_d["location"]))
    sim.register_cell(welding, location=tuple(weld_d["location"]))
    sim.register_cell(inspection, location=tuple(insp_d["location"]))

    return sim


def submit_jobs(sim: SimulationManager, cfg: Dict[str, Any]) -> None:
    for j in cfg["jobs"]:
        job = JobSpec(
            job_id=j["job_id"],
            part_id=j["part_id"],
            part_type=j["part_type"],
            process_route=list(j["process_route"]),
            recipe_overrides=dict(j.get("recipe_overrides", {})),
            priority=j.get("priority", "normal"),
        )
        sim.submit_job(job)


def print_report(sim: SimulationManager) -> None:
    print("\n" + "=" * 70)
    print(" FDW-OPS PoC-1 Simulation Report")
    print("=" * 70)
    print(f" Total simulated time : {sim.sim_time:.1f} s")
    print(f" Completed jobs       : {len(sim.orchestrator.completed_jobs)}")

    print("\n  Job histories:")
    for tr in sim.orchestrator.completed_jobs:
        ms = (tr.completed_at or sim.sim_time) - tr.started_at
        print(f"   - {tr.job.job_id} (part={tr.job.part_id})  "
              f"makespan={ms:6.1f}s  route={tr.history}")

    print("\n  Active jobs (incomplete):")
    if not sim.orchestrator.active_jobs:
        print("    (none)")
    for tr in sim.orchestrator.active_jobs.values():
        print(f"   - {tr.job.job_id}  current_loc={tr.current_location}  "
              f"step={tr.route_index}/{len(tr.job.process_route)}")

    print("\n  KPI summary:")
    summ = sim.kpi.summary()
    interesting = [
        "job_makespan_sec", "cycle_time", "quality_score",
        "quality_pass_rate", "predicted_gap_mm",
        "amr_delivery_count", "rack_stock_count",
    ]
    for m in interesting:
        if m in summ:
            s = summ[m]
            print(f"   - {m:25s}  count={int(s['count']):3d}  "
                  f"avg={s['avg']:7.3f}  min={s['min']:7.3f}  max={s['max']:7.3f}")

    print("=" * 70 + "\n")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path,
                   default=ROOT / "fdw_sim" / "config" / "poc1.yaml")
    p.add_argument("--mode", choices=["discrete", "isaac"], default=None,
                   help="시뮬레이션 백엔드 (None이면 config 사용)")
    p.add_argument("--headless", action="store_true",
                   help="Isaac Sim을 헤드리스로 실행 (mode=isaac일 때만 의미)")
    p.add_argument("--gui", action="store_true",
                   help="Isaac Sim GUI 모드 (mode=isaac일 때만 의미)")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = p.parse_args()

    setup_logging(level=args.log_level)

    cfg = load_config(args.config)

    headless_override = None
    if args.headless:
        headless_override = True
    elif args.gui:
        headless_override = False

    sim = build_simulation(cfg, mode_override=args.mode,
                           headless_override=headless_override)

    sim.start()
    try:
        submit_jobs(sim, cfg)
        sim.run_until_done(all_jobs_done=True, extra_idle_sec=2.0)
        print_report(sim)
    finally:
        sim.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
