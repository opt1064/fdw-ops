"""Inventory selected gripper in a separate USD stage; no workshop or motion."""
import argparse
import json
from pathlib import Path
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", help="Explicit candidate USD path/URI; no fallback to another variant")
    parser.add_argument("--tool-config", type=Path, default=ROOT / "fdw_sim/config/fanuc_tooling.yaml")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "fdw_sim/logs")
    args = parser.parse_args()
    # Bootstrap Isaac before importing pxr/omni or resolving installed asset root.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        from pxr import Usd
        from fdw_sim.visualization.robot_loader import get_isaac_assets_root
        from fdw_sim.visualization.tool_diagnostics import load_tool_configuration
        from fdw_sim.visualization.gripper_asset_inventory import inspect_gripper_stage
        source = args.usd
        try:
            config = load_tool_configuration(args.tool_config)
            if not source:
                subpath = config["tools"]["gripper"].get("candidate_usd_subpath")
                if not subpath:
                    raise ValueError("no gripper candidate USD selected")
                source = get_isaac_assets_root().rstrip("/") + "/" + subpath
            report = inspect_gripper_stage(Usd.Stage.Open(source))
        except Exception as exc:
            report = {"status": "FAILED", "errors": [f"{type(exc).__name__}: {exc}"],
                      "motion_commands_issued": False, "mounted_on_fanuc": False,
                      "physical_grasp_verified": False, "tcp_calibration_verified": False}
        report["source_usd"] = source
        args.output_dir.mkdir(parents=True, exist_ok=True)
        output = args.output_dir / (time.strftime("gripper_asset_inventory_%Y%m%d_%H%M%S_") + uuid4().hex[:8] + ".json")
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        print(f"[GRIPPER ASSET] saved: {output}")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2 if report["status"] == "FAILED" else 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
