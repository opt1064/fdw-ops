"""SimulationManager — 모든 컴포넌트(셀, 오케스트레이터, KPI Logger)를 묶어 step-loop 운영.

두 가지 백엔드를 지원한다:

    * "discrete"  : Isaac Sim 없이 순수 Python 루프로 실행 (Level 1 검증)
    * "isaac"     : Isaac Sim의 World/SimulationContext와 동기화 (Level 2+)

PoC-1은 "discrete" 모드로도 완전한 FDW-OPS 흐름이 검증된다.
Level 2 진입 시 동일 코드에서 mode="isaac"으로 전환하면 된다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional
import logging
import time

from fdw_sim.cells.base.cell_base import DistributedIntelligenceCell
from fdw_sim.cells.material.material_cell import (
    CellLocation,
    MaterialCell,
)
from fdw_sim.kpi.logger import KPILogger
from fdw_sim.messaging.bus import InMemoryBus, MessageBus
from fdw_sim.messaging.schemas import (
    CellState,
    JobSpec,
    KPIRecord,
)
from fdw_sim.orchestrator.orchestrator import (
    FDWOrchestrator,
    OrchestratorConfig,
)

logger = logging.getLogger(__name__)


# =============================================================================
# Config
# =============================================================================
@dataclass
class SimulationConfig:
    """SimulationManager 동작 파라미터."""
    mode: str = "discrete"            # "discrete" | "isaac"
    physics_hz: float = 60.0          # 시뮬레이션 물리 주기
    max_sim_time_sec: float = 300.0   # 최대 시뮬레이션 시간(초)
    realtime: bool = False            # True면 wall-clock 동기화 (디버깅 용)
    log_dir: Path = Path("./fdw_sim/logs")
    run_name: Optional[str] = None

    # Isaac Sim 모드 전용
    headless: bool = True
    stage_units_in_meters: float = 1.0
    livestream: int = 0  # 0=off, 1=Native, 2=WebRTC
    isaac_app_kwargs: Dict[str, Any] = field(default_factory=dict)  # type: ignore[arg-type]

    # 시각화 (mode == "isaac" 일 때만 사용)
    enable_visualization: bool = True
    transfer_anim_duration_sec: float = 2.0

    # Level 2.1: 실 로봇팔 옵션
    use_real_robot: bool = False        # True면 Franka/UR10 USD 로드, False면 placeholder 박스
    robot_name: str = "franka_panda"    # "franka_panda" | "ur10" | "franka_alt"
    enable_ik: bool = True              # IK 컨트롤러 활성화 (Lula 우선, 실패 시 휴리스틱 fallback)
    weld_path_offset_y: float = 0.25    # 용접 경로 길이 (m), 입력 버퍼 중심 ±offset
    weld_path_height: float = 0.05      # 부품 위 용접 높이 (m)

    # Level 2.2: RMPflow + 용접 스파크
    motion_mode: str = "auto"           # "auto" | "rmpflow" | "ik" | "heuristic"
                                          # auto: RMPflow → IK → heuristic 자동 fallback
    enable_sparks: bool = True          # 용접 스파크 파티클 활성화
    spark_rate: float = 30.0            # 스파크 방출 비율 (sparks/sec)
    spark_lifetime_sec: float = 0.4     # 개별 스파크 수명 (±20% jitter)
    rmpflow_register_obstacles: bool = True  # 작업대/부품을 RMPflow 장애물로 등록

    # Level 2.1: 렌더링 / RTX 안정성 옵션 (AGX Thor Blackwell GPU 호환)
    render_mode: str = "RaytracedLighting"   # "RaytracedLighting" | "PathTracing"
                                              # PathTracing은 NRD denoiser 필요 → Blackwell에서 셰이더 실패
    disable_nrd_denoiser: bool = True         # rtx.denoising.plugin (NRD) 비활성 — Blackwell 호환 패치
    suppress_rtx_log_spam: bool = True        # rtx.denoising 등 반복 에러 로그 억제

    # Level 2.1: GPU device-lost 회피용 안전 토글 (VkResult: ERROR_DEVICE_LOST 대응)
    # AGX Thor Blackwell에서 Aftermath/SceneCamera/Lighting setup 단계 GPU 크래시 회피
    skip_auto_camera: bool = False            # WorkshopVisualizer.build_scene 끝의 _auto_frame_camera 건너뜀
    disable_aftermath: bool = True            # NV Aftermath GPU crash dumper 비활성 (vk submit 부담↓)
    force_lighting_mode: Optional[str] = None # None=Kit 기본, "camera"|"stage"|"rig" 강제
    safe_mode: bool = False                   # True면 위 3개를 가장 보수적인 값으로 일괄 설정

    # Level 2.3: 셀별 실 USD 자산 사용 옵션 (placeholder fallback 자동)
    use_real_inspection_cam: bool = True      # inspection 셀에 실 카메라 prim
    use_real_amr: bool = True                 # AMR을 USD(NovaCarter 등)로 로드
    amr_asset_name: str = "nova_carter"       # asset_catalog 엔트리 이름
                                               # nova_carter | jetbot | iw_hub | iw_hub_static
                                               # nova_carter Props/* 404 이슈 시 jetbot/iw_hub_static 권장
    use_real_smart_rack: bool = True          # material 셀에 KLT bin USD rack
    smart_rack_asset_name: str = "klt_bin"    # asset_catalog 엔트리 이름
    use_real_forming_arm: bool = True         # forming 셀에 실 UR 로봇팔
    forming_robot_name: str = "ur10"          # robot_loader.ROBOT_CATALOG 키


# =============================================================================
# Manager
# =============================================================================
class SimulationManager:
    """전체 시뮬레이션의 라이프사이클 관리.

    사용 예 (discrete mode):

        bus = InMemoryBus()
        sim = SimulationManager(SimulationConfig(mode="discrete"), bus=bus)

        material = MaterialCell(...)
        welding = WeldingCell(...)
        inspection = InspectionCell(...)

        sim.register_material_cell(material, location=(0,0))
        sim.register_cell(welding, location=(5,0))
        sim.register_cell(inspection, location=(10,0))

        sim.start()
        sim.submit_job(JobSpec(...))
        sim.run_until_done()
        sim.stop()
    """

    def __init__(self, config: SimulationConfig,
                 bus: Optional[MessageBus] = None,
                 orchestrator_config: Optional[OrchestratorConfig] = None) -> None:
        self.config = config
        self.bus = bus or InMemoryBus()

        # 자체 컴포넌트
        self.kpi = KPILogger(log_dir=config.log_dir, run_name=config.run_name)
        self.kpi.attach(self.bus)

        self.orchestrator = FDWOrchestrator(self.bus, orchestrator_config)

        # 셀 레지스트리
        self.cells: Dict[str, DistributedIntelligenceCell] = {}
        self.material_cell: Optional[MaterialCell] = None

        # Isaac Sim 핸들 (mode == "isaac")
        self._isaac_app = None
        self._isaac_world = None
        self._isaac_launcher = None

        # 시각화 (mode == "isaac" + enable_visualization)
        self.visualizer = None  # WorkshopVisualizer (lazy)

        # 상태
        self._sim_time: float = 0.0
        self._dt: float = 1.0 / max(config.physics_hz, 1.0)
        self._running: bool = False

        # 사용자 정의 step hook
        self._step_hooks: List[Callable[[float, float], None]] = []

        # 셀 위치 캐시 (시각화에서 사용)
        self._cell_locations: Dict[str, tuple] = {}

    # =========================================================================
    # 셀 등록
    # =========================================================================
    def register_material_cell(self, cell: MaterialCell, location: tuple = (0.0, 0.0)) -> None:
        self.material_cell = cell
        self.cells[cell.cell_id] = cell
        cell.cell_locations[cell.cell_id] = CellLocation(cell.cell_id, location)
        self.orchestrator.register_cell(cell)
        # 시각화 좌표는 3D (z=0)
        self._cell_locations[cell.cell_id] = (location[0], location[1], 0.0)
        logger.info("[SIM] material cell %s registered @ %s", cell.cell_id, location)

    def register_cell(self, cell: DistributedIntelligenceCell, location: tuple = (0.0, 0.0)) -> None:
        if cell.cell_id in self.cells:
            raise ValueError(f"cell_id duplicated: {cell.cell_id}")
        self.cells[cell.cell_id] = cell
        if self.material_cell is not None:
            self.material_cell.register_cell(cell, CellLocation(cell.cell_id, location))
        self.orchestrator.register_cell(cell)
        self._cell_locations[cell.cell_id] = (location[0], location[1], 0.0)
        logger.info("[SIM] cell %s registered @ %s", cell.cell_id, location)

    def add_step_hook(self, hook: Callable[[float, float], None]) -> None:
        """매 step마다 호출될 함수를 등록 (signature: hook(dt, sim_time))."""
        self._step_hooks.append(hook)

    # =========================================================================
    # 라이프사이클
    # =========================================================================
    def start(self) -> None:
        if self._running:
            return

        if self.config.mode == "isaac":
            self._start_isaac()
        else:
            logger.info("[SIM] starting in DISCRETE mode (physics_hz=%.0f)", self.config.physics_hz)

        # 모든 셀 boot
        for cell in self.cells.values():
            cell.boot()

        self._running = True

    def _start_isaac(self) -> None:
        """Isaac Sim 5.x / Isaac Lab 백엔드 초기화.

        Isaac Sim 5.x에서는 두 가지 방법으로 SimulationApp을 시작할 수 있다.
        1) isaacsim 메타 패키지가 설치된 경우: from isaacsim import SimulationApp
        2) Isaac Lab만 설치된 경우: isaaclab.app.AppLauncher 사용 (권장)
        """
        # 0) safe_mode 일괄 적용 — AGX Thor에서 GPU device lost 회피
        if self.config.safe_mode:
            self.config.skip_auto_camera = True
            self.config.disable_aftermath = True
            self.config.disable_nrd_denoiser = True
            self.config.suppress_rtx_log_spam = True
            self.config.render_mode = "RaytracedLighting"
            if self.config.force_lighting_mode is None:
                self.config.force_lighting_mode = "camera"
            logger.warning("[SIM] SAFE_MODE engaged — "
                           "skip_auto_camera=ON, disable_aftermath=ON, "
                           "force_lighting_mode=%s, render=RaytracedLighting",
                           self.config.force_lighting_mode)

        # 0.1) RTX denoiser + Aftermath 사전 차단 (AGX Thor Blackwell GPU 호환)
        #    SimulationApp 시작 전에 환경변수와 stderr 필터를 미리 잡아야
        #    rtx.denoising.plugin 셰이더 컴파일 에러 로그 폭주를 막을 수 있다.
        self._pre_app_rtx_guard()

        # 1) SimulationApp 가장 먼저 생성 (Carbonite 요구사항)
        # carb experimental settings를 launch kwargs로 미리 주입
        extra_args = self._build_app_launch_args()

        # Isaac Lab의 AppLauncher가 설치돼 있는지만 먼저 확인한다 (import 실패 =
        # isaaclab 패키지 자체가 없는 경우에만 fallback으로 넘어가야 한다).
        # AppLauncher(...) 생성 자체는 이 try에 넣지 않는다 — Kit 부팅이 도중에
        # 실패하면 (예: 확장 dependency solver 실패) SimulationApp.__init__ 내부의
        # `from omni.kit.usd import layers` 등에서 ModuleNotFoundError가 올라오는데,
        # 이것도 ImportError의 서브클래스라서 예전 코드는 "isaaclab 미설치"로 착각하고
        # 이미 절반쯤 부팅되다 만 같은 프로세스 안에서 SimulationApp()을 다시
        # 시도했다. Kit/Carbonite는 프로세스당 한 번만 안전하게 초기화되므로 이
        # 재시도는 항상 같은 에러로 다시 죽고, 진짜 원인(첫 실패)은 로그에 묻혀
        # 사라진다. 아래처럼 import 여부만 분기하면 AppLauncher 부팅 중 실패는
        # 마스킹되지 않고 그대로 위로 올라가 원인을 바로 알 수 있다.
        try:
            from isaaclab.app import AppLauncher  # type: ignore
        except ImportError:
            AppLauncher = None  # type: ignore[assignment]

        if AppLauncher is not None:
            launcher_args = {
                "headless": self.config.headless,
                "livestream": self.config.livestream,
                **extra_args,
                **self.config.isaac_app_kwargs,
            }
            self._isaac_launcher = AppLauncher(launcher_args)
            self._isaac_app = self._isaac_launcher.app
            logger.info("[SIM] Isaac Sim launched via Isaac Lab AppLauncher "
                        "(headless=%s, livestream=%d)",
                        self.config.headless, self.config.livestream)
        else:
            # Fallback: isaacsim 메타 패키지 사용 (isaaclab이 아예 미설치된 경우만)
            from isaacsim import SimulationApp  # type: ignore
            app_kwargs = {
                "headless": self.config.headless,
                **extra_args,
                **self.config.isaac_app_kwargs,
            }
            if self.config.livestream > 0:
                app_kwargs["livestream"] = self.config.livestream
            self._isaac_app = SimulationApp(app_kwargs)
            logger.info("[SIM] Isaac Sim launched via SimulationApp "
                        "(headless=%s, livestream=%d)",
                        self.config.headless, self.config.livestream)

        # 1.5) SimulationApp 직후 — World 생성 전에 RTX/log 설정 적용
        #      (denoiser plugin이 첫 프레임 렌더 전에 비활성되어야 함)
        self._apply_rtx_settings()

        # 2) 그 다음에 World import / 생성
        try:
            from isaacsim.core.api import World  # type: ignore
        except ImportError:
            # Isaac Sim 4.x 호환 경로
            from omni.isaac.core import World  # type: ignore

        self._isaac_world = World(
            stage_units_in_meters=self.config.stage_units_in_meters,
            physics_dt=self._dt,
            rendering_dt=self._dt,
        )
        self._isaac_world.scene.add_default_ground_plane()
        self._isaac_world.reset()

        # 2.5) World 생성 후에도 한 번 더 적용 (일부 키는 reset에서 덮어쓰임)
        self._apply_rtx_settings()

        # 3) 시각화 빌드 (옵션)
        if self.config.enable_visualization:
            self._build_visualization()

    def _pre_app_rtx_guard(self) -> None:
        """SimulationApp 시작 전 환경변수 + stderr 필터 설치.

        SimulationApp이 인스턴스화되는 순간 carb logger가 stdout/stderr로
        직접 로그를 쏘기 때문에, Python logger 레벨로는 막을 수 없다.
        따라서 다음 두 가지를 미리 처리한다:
          1) 환경변수 — Kit가 시작 시 읽는 RTX 관련 기본값
          2) stderr 필터 스레드 — 'rtx.denoising' 포함 라인을 drop
        """
        if not (self.config.disable_nrd_denoiser
                or self.config.suppress_rtx_log_spam
                or self.config.disable_aftermath):
            return

        import os
        # 1) 환경변수로 carb settings 사전 주입 (Kit 5.x 지원)
        env_overrides = {
            "CARB_APP_PATH": os.environ.get("CARB_APP_PATH", ""),
            # 렌더 모드
            "RTX_RENDERMODE": self.config.render_mode,
            # NRD denoiser 차단용 추가 키
            "RTX_DENOISING_ENABLED": "0",
            "RTX_NEWDENOISER_ENABLED": "0",
        }
        # Aftermath GPU crash dumper 비활성
        # — Aftermath는 매 vk submit에 콜백을 끼워 넣어 device lost 확률을 높일 수 있음
        if self.config.disable_aftermath:
            env_overrides.update({
                "NVDA_AFTERMATH": "0",
                "RTX_AFTERMATH_ENABLED": "0",
                "NSIGHT_AFTERMATH_ENABLED": "0",
                # NGX/Optix 자체 비활성 (Blackwell에서 미지원 — 셰이더 로드 자체를 건너뜀)
                "RTX_NGX_ENABLED": "0",
                "OMNI_KIT_DISABLE_GPU_FALLBACK": "0",
                # breakpad crash reporter (Aftermath 동반) — symbol upload 부담 제거
                "OMNI_KIT_CRASH_REPORT_DISABLED": "1",
            })
            logger.info("[SIM] Aftermath / NGX / breakpad disabled via env "
                        "(Blackwell device-lost 회피)")
        for k, v in env_overrides.items():
            if v:
                os.environ.setdefault(k, v)

        # 2) stderr 필터 — 'rtx.denoising' 라인을 화면에서 차단
        if self.config.suppress_rtx_log_spam:
            self._install_stderr_filter([
                "rtx.denoising",
                "PackForNRD",
                "rtx/nrd/",
                # USD xform op order 경고 (이미 _enforce_xform_order로 예방하지만 안전망)
                "Incompatible xformOpOrder",
                # carb 키 타입 경고 (예전 코드에서 string으로 잘못 설정한 경우)
                "getStringRawInternal",
                # NGX/Optix 초기화 실패 경고 (Blackwell에서 정상)
                "NGX isn't enabled",
                "Failed to create NGX context",
                "Failed to create an Optix",
            ])

    def _install_stderr_filter(self, drop_substrings: List[str]) -> None:
        """OS fd=2(stderr) 레벨에서 라인 필터 설치 — C++ carb logger도 차단.

        carb logger는 C++ 레이어에서 fd=2에 직접 write 하므로 Python
        sys.stderr 교체로는 막을 수 없다. 따라서 다음과 같이 처리한다:

          1) os.pipe()로 새 파이프 생성
          2) os.dup2(pipe_write, 2) — 원래 stderr fd=2를 파이프 쓰기단으로 교체
          3) 백그라운드 스레드에서 파이프를 라인 단위로 읽어 필터링 후
             원본 stderr (백업해둔 fd)로 다시 write

        이렇게 하면 Python 코드든, C++ 라이브러리든, 모든 fd=2 write가
        필터를 거치게 된다.
        """
        import os
        import sys
        import threading

        if getattr(self, "_stderr_filter_installed", False):
            return

        # 1) 원본 stderr fd 백업 + 새 파이프 생성
        try:
            original_stderr_fd = os.dup(2)   # fd=2 백업
            pipe_read, pipe_write = os.pipe()
            os.dup2(pipe_write, 2)            # fd=2 → 파이프 쓰기단
            os.close(pipe_write)
        except OSError as e:
            logger.warning("[SIM] fd-level stderr filter failed: %s", e)
            return

        # 원본 fd → Python file object (line buffered)
        original_stderr = os.fdopen(original_stderr_fd, "w", buffering=1,
                                     encoding="utf-8", errors="replace")
        pipe_reader = os.fdopen(pipe_read, "r", buffering=1,
                                 encoding="utf-8", errors="replace")

        drops = list(drop_substrings)
        dropped_counter = {"n": 0}

        def _filter_loop() -> None:
            try:
                for line in pipe_reader:
                    if any(d in line for d in drops):
                        dropped_counter["n"] += 1
                        continue
                    original_stderr.write(line)
                    original_stderr.flush()
            except Exception:
                pass

        t = threading.Thread(target=_filter_loop, name="stderr-filter",
                              daemon=True)
        t.start()

        # Python sys.stderr도 동일 fd를 사용하도록 갱신
        try:
            sys.stderr.flush()
        except Exception:
            pass

        self._stderr_filter_installed = True
        self._stderr_filter_state = {
            "original_fd": original_stderr_fd,
            "thread": t,
            "drops": drops,
            "dropped_counter": dropped_counter,
        }
        # 이 메시지는 필터 설치 이후 fd=2로 직접 가는 게 아니라
        # python logger를 통해 가므로 그대로 보임
        logger.info("[SIM] fd-level stderr filter installed (drop: %s)", drops)

    def _build_app_launch_args(self) -> Dict[str, Any]:
        """SimulationApp / AppLauncher에 전달할 추가 인자.

        Kit는 dict 키를 '--/rtx/...' 형식 CLI 인자로 변환해 carb settings에
        주입한다. 따라서 disable_nrd_denoiser=True이면 여기서 미리 settings를
        지정할 수 있다.
        """
        if not self.config.disable_nrd_denoiser:
            return {}

        # Kit launch kwargs는 일반적으로 정해진 키만 인식하므로
        # carb settings 인젝션은 env 변수 + _apply_rtx_settings에 의존.
        # 안전한 키만 전달.
        return {
            "renderer": "RayTracedLighting" if self.config.render_mode == "RaytracedLighting"
                        else "PathTracing",
        }

    def _apply_rtx_settings(self) -> None:
        """RTX 렌더러 설정 — Blackwell GPU(AGX Thor)에서 NRD denoiser 셰이더
        컴파일 실패로 매 프레임 에러 로그가 쏟아지는 문제를 회피한다.

        주요 처리:
            1) 렌더 모드를 PathTracing → RaytracedLighting으로 (NRD 불필요)
            2) rtx.denoising.* 플러그인 자체를 비활성
            3) carb 로그 필터로 반복 에러 출력 억제
        """
        try:
            import carb  # type: ignore
            settings = carb.settings.get_settings()
        except Exception as e:
            logger.debug("[SIM] carb settings unavailable: %s", e)
            return

        # carb settings 헬퍼 — 키 타입 오류를 individually try/except
        def _try_set(key: str, value):
            try:
                settings.set(key, value)
                return True
            except Exception as e:
                logger.debug("[SIM] settings.set(%s, %r) failed: %s", key, value, e)
                return False

        # 1) 렌더 모드 (RaytracedLighting은 NRD를 사용하지 않음)
        _try_set("/rtx/rendermode", self.config.render_mode)
        _try_set("/rtx/pathtracing/enabled",
                 self.config.render_mode != "RaytracedLighting")
        logger.info("[SIM] RTX render mode = %s", self.config.render_mode)

        # 2) NRD denoiser 비활성 (Blackwell 셰이더 호환성 회피)
        if self.config.disable_nrd_denoiser:
            for key, val in [
                ("/rtx/post/dlss/execMode", 0),
                ("/rtx-transient/dldenoiser/enabled", False),
                ("/rtx-transient/denoiser/enabled", False),
                ("/rtx/newDenoiser/enabled", False),
                ("/rtx/directLighting/sampledLighting/enabled", False),
                ("/rtx/denoising/enabled", False),
                ("/rtx/denoising/nrd/enabled", False),
                ("/rtx/pathtracing/denoiser/enabled", False),
                ("/rtx/pathtracing/optixDenoiser/enabled", False),
                # 추가: NGX/Optix 자체 disable (셰이더 컴파일 자체를 건너뜀)
                ("/rtx-transient/ngx/enabled", False),
                ("/rtx/raytracing/lightcache/spatialCache/enabled", False),
            ]:
                _try_set(key, val)
            logger.info("[SIM] NRD/path-tracing denoiser disabled "
                        "(Blackwell GPU compatibility)")

        # 3) Aftermath / NGX / breakpad 비활성 (GPU device-lost 회피)
        if self.config.disable_aftermath:
            for key, val in [
                ("/rtx/aftermath/enabled", False),
                ("/rtx/aftermath/shaderHashAttachment/enabled", False),
                ("/app/renderer/enableGpuCrashDumping", False),
                ("/app/enableCrashReporting", False),
                ("/app/runLoops/main/manualModeEnabled", False),
                ("/persistent/app/captureFrame/captureMode", 0),
                # NGX / Optix
                ("/rtx-transient/ngx/enabled", False),
                ("/rtx/ngx/enabled", False),
                # Vulkan resource upload 안정화
                ("/rtx/resourcemanager/maxStagedUploadMB", 64),
                ("/rtx/resourcemanager/texturestreaming/async", False),
            ]:
                _try_set(key, val)
            logger.info("[SIM] Aftermath + NGX + async upload disabled "
                        "(Blackwell device-lost 회피)")

        # 4) Lighting menu mode 강제 (GPU crash 마지막 명령이 SetLightingMenuMode였음)
        if self.config.force_lighting_mode:
            mode_map = {"camera": 0, "stage": 1, "rig": 2}
            mode_val = mode_map.get(self.config.force_lighting_mode, 0)
            for key, val in [
                ("/rtx/sceneDb/ambientLightIntensity", 0.3),
                ("/persistent/app/viewport/displayOptions", 31951),
                ("/persistent/app/stage/upAxis", "Z"),
                # Kit 5.x 라이팅 메뉴 모드
                ("/persistent/app/viewport/Viewport/Viewport0/lightingMode",
                 self.config.force_lighting_mode),
                ("/app/renderer/skipMaterialLoading", False),
            ]:
                _try_set(key, val)
            logger.info("[SIM] lighting mode forced to '%s' (idx=%d) "
                        "— GPU crash 직전 'SetLightingMenuModeCommand' 회피",
                        self.config.force_lighting_mode, mode_val)

        # 5) 로그 스팸 억제 — carb 로그 레벨 (int)로 설정
        # [Error] [carb.dictionary.plugin] getStringRawInternal: item ... is not a string
        # → /log/channels/.../level 키는 string이 아니라 int (carb.logging.LEVEL_*)
        if self.config.suppress_rtx_log_spam:
            # carb log level 상수: VERBOSE=-2, INFO=-1, WARN=0, ERROR=1, FATAL=2
            LEVEL_FATAL = 2
            for chan in [
                "rtx.denoising",
                "rtx.denoising.plugin",
                "rtx-transient.denoiser",
                "rtx.optixdenoising",
                "rtx.optixdenoising.plugin",
                "gpu.foundation.plugin",
                "carb.graphics-vulkan.plugin",
            ]:
                # 채널 자체를 disable + level 모두 시도
                _try_set(f"/log/channels/{chan}/enabled", False)
                _try_set(f"/log/channels/{chan}/level", LEVEL_FATAL)
            logger.info("[SIM] rtx.denoising log channels muted")

    def _build_visualization(self) -> None:
        """Isaac Sim 시작 후 USD 시각화 구성."""
        try:
            from fdw_sim.visualization.workshop_visualizer import (
                WorkshopVisualizer, WorkshopVizConfig,
            )
        except Exception as e:
            logger.warning("[SIM] visualization disabled (import failed: %s)", e)
            return

        viz_cfg = WorkshopVizConfig(
            transfer_duration_sec=self.config.transfer_anim_duration_sec,
            use_real_robot=self.config.use_real_robot,
            robot_name=self.config.robot_name,
            enable_ik=self.config.enable_ik,
            weld_path_offset_y=self.config.weld_path_offset_y,
            weld_path_height=self.config.weld_path_height,
            skip_auto_camera=self.config.skip_auto_camera,
            # Level 2.2
            motion_mode=self.config.motion_mode,
            enable_sparks=self.config.enable_sparks,
            spark_rate=self.config.spark_rate,
            spark_lifetime_sec=self.config.spark_lifetime_sec,
            rmpflow_register_obstacles=self.config.rmpflow_register_obstacles,
            # Level 2.3
            use_real_inspection_cam=self.config.use_real_inspection_cam,
            use_real_amr=self.config.use_real_amr,
            amr_asset_name=self.config.amr_asset_name,
            use_real_smart_rack=self.config.use_real_smart_rack,
            smart_rack_asset_name=self.config.smart_rack_asset_name,
            use_real_forming_arm=self.config.use_real_forming_arm,
            forming_robot_name=self.config.forming_robot_name,
        )
        self.visualizer = WorkshopVisualizer(bus=self.bus, config=viz_cfg)

        # 셀 등록 (이미 register_cell 단계에서 _cell_locations에 저장됨)
        for cell_id, cell in self.cells.items():
            ctype = cell.config.cell_type.value if hasattr(cell.config, "cell_type") else "default"
            pos = self._cell_locations.get(cell_id, (0.0, 0.0, 0.0))
            self.visualizer.register_cell(cell_id, cell_type=ctype, position=pos)

        # AMR 등록 (MaterialCell의 AMR들)
        if self.material_cell is not None:
            for i, amr in enumerate(self.material_cell.amrs):
                amr_id = getattr(amr, "amr_id", f"AMR_{i:02d}")
                # AMR을 MaterialCell 옆에 배치
                mat_pos = self._cell_locations.get(self.material_cell.cell_id, (0.0, 0.0, 0.0))
                amr_pos = (mat_pos[0] - 1.5 + i * 0.7, mat_pos[1] - 1.5, 0.0)
                self.visualizer.register_amr(amr_id, position=amr_pos)
            self.visualizer.attach_material_cell(self.material_cell)

        # USD 스테이지에 빌드
        self.visualizer.build_scene()

        # transfer 이벤트 구독 — 부품 이동 애니메이션 트리거
        self._setup_transfer_visualization()

    def _setup_transfer_visualization(self) -> None:
        """MaterialCell의 transfer 완료 이벤트를 시각화에 연결."""
        if self.visualizer is None:
            return

        from fdw_sim.messaging.bus import Topics

        def on_transfer(msg) -> None:
            """MATERIAL_TRANSFER 토픽 콜백."""
            try:
                # MaterialTransferCommand는 dataclass
                part_id = getattr(msg, "part_id", None)
                from_cell = getattr(msg, "from_cell", None)
                to_cell = getattr(msg, "to_cell", None)
                if part_id and to_cell:
                    self.visualizer.transfer_part(
                        part_id,
                        from_cell or self.material_cell.cell_id,
                        to_cell,
                    )
            except Exception:
                logger.exception("transfer visualization hook failed")

        self.bus.subscribe(Topics.MATERIAL_TRANSFER, on_transfer)

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        self.kpi.close()
        if self.config.mode == "isaac" and self._isaac_app is not None:
            self._isaac_app.close()
        self.bus.shutdown()
        logger.info("[SIM] stopped (sim_time=%.1fs)", self._sim_time)

    # =========================================================================
    # Step loop
    # =========================================================================
    def step(self) -> None:
        """단일 시뮬레이션 틱."""
        dt = self._dt

        # 1) Isaac Sim 모드: 물리 step
        if self.config.mode == "isaac" and self._isaac_world is not None:
            self._isaac_world.step(render=not self.config.headless)

        # 2) 모든 셀 step
        for cell in self.cells.values():
            cell.step(dt, self._sim_time)

        # 3) 오케스트레이터 step
        self.orchestrator.step(dt, self._sim_time)

        # 4) 시각화 업데이트 (Isaac mode + visualization enabled)
        if self.visualizer is not None:
            try:
                self.visualizer.update(dt, self._sim_time)
            except Exception:
                logger.exception("visualizer update failed")

        # 5) 사용자 hook
        for hook in self._step_hooks:
            try:
                hook(dt, self._sim_time)
            except Exception:
                logger.exception("step hook failed")

        self._sim_time += dt

    def run_until_done(self,
                       all_jobs_done: bool = True,
                       extra_idle_sec: float = 2.0) -> None:
        """모든 active job이 완료되거나 max_sim_time에 도달할 때까지 step.

        Args:
            all_jobs_done: True면 active_jobs가 비고 모든 셀이 IDLE이 될 때까지 진행
            extra_idle_sec: 모든 작업 완료 후 추가로 진행할 시간(상태 안정화)
        """
        idle_since: Optional[float] = None
        wall_start = time.time()

        while self._running and self._sim_time < self.config.max_sim_time_sec:
            self.step()

            if all_jobs_done:
                if not self.orchestrator.active_jobs and self._all_cells_quiescent():
                    if idle_since is None:
                        idle_since = self._sim_time
                    elif self._sim_time - idle_since >= extra_idle_sec:
                        break
                else:
                    idle_since = None

            if self.config.realtime:
                target_wall = wall_start + self._sim_time
                lag = target_wall - time.time()
                if lag > 0:
                    time.sleep(lag)

        logger.info("[SIM] run finished (sim_time=%.1fs, completed_jobs=%d)",
                    self._sim_time, len(self.orchestrator.completed_jobs))

    def _all_cells_quiescent(self) -> bool:
        """모든 일반 셀이 IDLE 상태이고, MaterialCell의 transfer_queue/AMR이 비어있는지."""
        for cell in self.cells.values():
            if isinstance(cell, MaterialCell):
                if cell.transfer_queue:
                    return False
                if any(amr.busy for amr in cell.amrs):
                    return False
            else:
                if cell.fsm.state not in (CellState.IDLE,):
                    return False
                if cell.input_buffer.occupied or cell.output_buffer.occupied:
                    return False
        return True

    # =========================================================================
    # Job 투입 헬퍼
    # =========================================================================
    def submit_job(self, job: JobSpec) -> None:
        if self.material_cell is None:
            raise RuntimeError("material cell not registered")
        # 부품을 smart rack에 입고
        self.material_cell.stock_part(job.part_id, job.part_type)
        # 오케스트레이터에 Job 등록
        self.orchestrator.submit_job(job)

    # =========================================================================
    # 디버깅
    # =========================================================================
    @property
    def sim_time(self) -> float:
        return self._sim_time
