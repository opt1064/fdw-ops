"""FDW-OPS Virtual Workshop — Isaac Sim 기반 분산지능 셀 시뮬레이터.

Architecture:
    [FDW-OPS Orchestrator] (전체 공정 운영 AI)
            ↓ Middleware (ROS 2 / DDS / In-Memory Bus)
    [분산지능 셀들] Material / Welding / Forming / WAAM / Machining / Inspection
            ↓
    [Isaac Sim] (물리·로봇·센서·공간 검증)
"""

__version__ = "0.1.0"
