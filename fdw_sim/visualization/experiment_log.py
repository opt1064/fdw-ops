"""Durable partial evidence and error output before SimulationApp teardown."""
import json
import os
from pathlib import Path
import sys
import time
import traceback
from uuid import uuid4


class ExperimentLog:
    def __init__(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("gripper_motion_%Y%m%d_%H%M%S_")+uuid4().hex[:8]
        self.output = directory / (stem+".json")
        self.events = directory / (stem+".events.jsonl")
        self.trace = directory / (stem+".traceback.txt")
        self.event("RUN_STARTED", status="IN_PROGRESS")

    def event(self, kind, **data):
        row = {"event": kind, "wall_time_unix_sec": time.time(), **data}
        text = json.dumps(row, ensure_ascii=False, allow_nan=False)
        with self.events.open("a", encoding="utf-8") as stream:
            stream.write(text+"\n")
            stream.flush()
            os.fsync(stream.fileno())

    def exception(self, context, exc):
        message = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
        print(f"[GRIPPER ERROR] {context}\n{message}", file=sys.stderr, flush=True)
        try:
            with self.trace.open("a", encoding="utf-8") as stream:
                stream.write(context+"\n"+message+"\n")
                stream.flush()
                os.fsync(stream.fileno())
            self.event("EXCEPTION", context=context, error=f"{type(exc).__name__}: {exc}")
        except Exception as diagnostic_error:
            print(f"[GRIPPER ERROR] Could not persist exception: {diagnostic_error}", file=sys.stderr, flush=True)

    def save(self, report):
        try:
            content = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
        except (TypeError, ValueError) as exc:
            self.exception("result serialization", exc)
            # Never discard the original motion failure because a diagnostic field is invalid.
            fallback = {"status": "FAILED", "physical_grasp_verified": False, "transfer_verified": False,
                        "target_placement_verified": False, "report_incomplete": True,
                        "errors": [str(e) for e in report.get("errors", [])]+[f"result serialization: {exc}"],
                        "events_file": str(self.events), "traceback_file": str(self.trace)}
            self._atomic_write(json.dumps(fallback, ensure_ascii=False, indent=2, allow_nan=False))
            raise
        self._atomic_write(content)
        self.event("REPORT_SAVED", status=report["status"], output=str(self.output))
        return self.output

    def _atomic_write(self, content):
        temporary = self.output.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.output)
