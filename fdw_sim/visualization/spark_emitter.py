"""WeldingSparkEmitter — Level 2.2 용접 스파크 파티클 시스템.

PointInstancer 기반 GPU-friendly 스파크 풀.
- TCP(엔드 이펙터) 위치를 추적하여 weld 단계에서 스파크 방출
- 중력/lifetime/속도 spread를 numpy로 적분
- USD PointInstancer.positions/orientations/scales/protoIndices에 매 프레임 푸시
- Isaac Sim 미설치 환경에서도 import 가능 (lazy import)

설계 원칙
---------
* SceneBuilder가 USD 스테이지를 보유하므로, 이 모듈은
  ``add_sparks_pointinstancer(stage, parent_path)`` 헬퍼와
  ``WeldingSparkEmitter`` 두 가지를 export.
* Emitter는 **소유한 PointInstancer 경로**만 알면 됨.
* update(dt)는 (1) 죽은 입자 회수 (2) 새 스파크 spawn (3) 활성 입자 적분 (4) USD push.

사용 예 (workshop_visualizer에서)::

    emitter = WeldingSparkEmitter(stage, cell_path + "/Sparks", SparkEmitterConfig())
    emitter.set_tcp_position((x, y, z))
    emitter.set_active(True)   # weld phase 진입
    emitter.update(dt)         # 매 sim step
    emitter.set_active(False)  # retreat phase
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import logging
import math

logger = logging.getLogger(__name__)


# =============================================================================
# 설정
# =============================================================================
@dataclass
class SparkEmitterConfig:
    """용접 스파크 emitter 파라미터."""

    # 풀 크기 — 동시에 화면에 존재할 수 있는 최대 스파크 수
    pool_size: int = 96

    # 초당 새 스파크 생성 비율
    spark_rate: float = 30.0

    # 입자 lifetime (s) — 평균값, ±20% 랜덤
    lifetime_sec: float = 0.4

    # 초기 속도 (m/s) min/max
    initial_speed_min: float = 0.6
    initial_speed_max: float = 1.6

    # 분사 spread (도) — 0=완전 직선(=normal 방향), 180=전방향
    spread_angle_deg: float = 60.0

    # 분사 방향 (월드 좌표; 보통 +Z 위쪽)
    emit_direction: Tuple[float, float, float] = (0.0, 0.0, 1.0)

    # 중력 (m/s²; 기본 -9.81 z)
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)

    # 입자 크기 (m); USD scale로 적용
    spark_size: float = 0.012

    # 색상 (오렌지/노랑 — emissive 효과 의도)
    spark_color: Tuple[float, float, float] = (1.0, 0.55, 0.1)

    # 입자가 바닥(z<=floor_z)에 닿으면 소멸
    floor_z: float = -10.0  # 기본은 사실상 비활성

    # 디버그 로그
    verbose: bool = False


# =============================================================================
# USD 헬퍼 — PointInstancer 생성
# =============================================================================
def add_sparks_pointinstancer(
    stage,
    parent_path: str,
    pool_size: int,
    spark_size: float,
    spark_color: Tuple[float, float, float],
) -> str:
    """PointInstancer + sphere prototype을 USD 스테이지에 생성.

    Args:
        stage: pxr.Usd.Stage
        parent_path: 인스턴서를 생성할 부모 prim 경로 (예: cell_root)
        pool_size: 프리미티브 슬롯 수
        spark_size: 스파크 반지름 (m)
        spark_color: displayColor

    Returns:
        PointInstancer prim path
    """
    from pxr import Usd, UsdGeom, Sdf, Gf, Vt  # type: ignore

    instancer_path = f"{parent_path}/Sparks"
    instancer = UsdGeom.PointInstancer.Define(stage, instancer_path)

    # Prototype — 작은 sphere
    proto_path = f"{instancer_path}/Proto"
    UsdGeom.Xform.Define(stage, proto_path)
    sphere_path = f"{proto_path}/Sphere"
    sphere = UsdGeom.Sphere.Define(stage, sphere_path)
    sphere.CreateRadiusAttr(float(spark_size))
    sphere.GetDisplayColorAttr().Set([Gf.Vec3f(*spark_color)])

    # prototypes relationship
    instancer.CreatePrototypesRel().SetTargets([Sdf.Path(proto_path)])

    # 초기 빈 배열 (모두 비활성 = 화면 밖)
    instancer.CreateProtoIndicesAttr().Set(Vt.IntArray([]))
    instancer.CreatePositionsAttr().Set(Vt.Vec3fArray([]))
    instancer.CreateScalesAttr().Set(Vt.Vec3fArray([]))
    instancer.CreateOrientationsAttr().Set(Vt.QuathArray([]))

    return instancer_path


# =============================================================================
# Emitter 본체
# =============================================================================
class WeldingSparkEmitter:
    """용접 스파크 파티클 emitter (PointInstancer 기반).

    Lifecycle:
        ctor → set_tcp_position() → set_active(True) → update(dt) ... → set_active(False)
    """

    def __init__(
        self,
        stage,
        parent_path: str,
        config: Optional[SparkEmitterConfig] = None,
    ) -> None:
        self.config = config or SparkEmitterConfig()
        self._stage = stage
        self._parent_path = parent_path

        # numpy 지연 import (테스트 환경에서도 가능)
        import numpy as np  # type: ignore
        self._np = np

        # USD lazy import
        from pxr import UsdGeom, Gf, Vt  # type: ignore
        self._UsdGeom = UsdGeom
        self._Gf = Gf
        self._Vt = Vt

        # PointInstancer 생성
        self._instancer_path = add_sparks_pointinstancer(
            stage,
            parent_path,
            pool_size=self.config.pool_size,
            spark_size=self.config.spark_size,
            spark_color=self.config.spark_color,
        )
        self._instancer = UsdGeom.PointInstancer(stage.GetPrimAtPath(self._instancer_path))

        # 입자 상태 배열 (numpy)
        N = self.config.pool_size
        self._pos = np.zeros((N, 3), dtype=np.float32)
        self._vel = np.zeros((N, 3), dtype=np.float32)
        self._age = np.zeros((N,), dtype=np.float32)        # current age
        self._life = np.zeros((N,), dtype=np.float32)       # lifetime
        self._alive = np.zeros((N,), dtype=bool)

        # TCP 추적
        self._tcp = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._tcp_prev = self._tcp.copy()

        # 활성화
        self._active = False
        self._spawn_accumulator = 0.0  # 분수 spawn 잔량

        # RNG
        self._rng = np.random.default_rng(seed=12345)

        if self.config.verbose:
            logger.info(
                "[VIS] WeldingSparkEmitter ready @ %s (pool=%d, rate=%.1f/s, life=%.2fs)",
                self._instancer_path,
                self.config.pool_size,
                self.config.spark_rate,
                self.config.lifetime_sec,
            )

    # ------------------------------------------------------------------ API
    def get_instancer_path(self) -> str:
        return self._instancer_path

    def set_tcp_position(self, position: Tuple[float, float, float]) -> None:
        """엔드 이펙터 (=새 스파크 spawn 지점) 위치 갱신."""
        self._tcp_prev = self._tcp.copy()
        self._tcp = self._np.asarray(position, dtype=self._np.float32)

    def set_active(self, active: bool) -> None:
        """스파크 방출 on/off (이미 활성 중인 입자는 자연 소멸)."""
        self._active = bool(active)

    def is_active(self) -> bool:
        return self._active

    def alive_count(self) -> int:
        return int(self._alive.sum())

    def clear(self) -> None:
        """전체 풀 즉시 비활성화 + USD 빈 배열 push."""
        self._alive[:] = False
        self._age[:] = 0.0
        self._spawn_accumulator = 0.0
        self._push_to_usd()

    # ------------------------------------------------------------- 메인 루프
    def update(self, dt: float) -> None:
        """시뮬레이션 1 step. 새 입자 spawn → 적분 → 만료 회수 → USD push."""
        if dt <= 0.0:
            return

        np_ = self._np

        # 1) 적분 — 활성 입자만
        mask = self._alive
        if mask.any():
            g = np_.asarray(self.config.gravity, dtype=np_.float32)
            self._vel[mask] += g * dt
            self._pos[mask] += self._vel[mask] * dt
            self._age[mask] += dt

            # 만료/바닥 충돌
            expired = mask & (self._age >= self._life)
            below = mask & (self._pos[:, 2] <= self.config.floor_z)
            killed = expired | below
            if killed.any():
                self._alive[killed] = False

        # 2) 새 스파크 spawn
        if self._active:
            self._spawn_accumulator += self.config.spark_rate * dt
            n_spawn = int(self._spawn_accumulator)
            self._spawn_accumulator -= n_spawn
            if n_spawn > 0:
                self._spawn(n_spawn)

        # 3) USD push
        self._push_to_usd()

    # ------------------------------------------------------------- 내부 도구
    def _spawn(self, n: int) -> None:
        """비활성 슬롯을 찾아 n개 입자를 생성."""
        np_ = self._np
        free = np_.where(~self._alive)[0]
        if free.size == 0:
            return
        n = min(n, free.size)
        idx = free[:n]

        # 위치: TCP에 약간의 jitter (∼1mm)
        jitter = self._rng.normal(scale=0.001, size=(n, 3)).astype(np_.float32)
        self._pos[idx] = self._tcp[None, :] + jitter

        # 속도: emit_direction을 axis로 spread_angle_deg 콘 내에서 균일
        direction = np_.asarray(self.config.emit_direction, dtype=np_.float32)
        norm = float(np_.linalg.norm(direction))
        if norm < 1e-6:
            direction = np_.array([0.0, 0.0, 1.0], dtype=np_.float32)
        else:
            direction = direction / norm

        spread_rad = math.radians(self.config.spread_angle_deg)
        # 콘 내부 균일 샘플링 (z-up 콘에서 샘플 → emit_direction 으로 회전)
        cos_half = math.cos(spread_rad / 2.0)
        u = self._rng.uniform(0.0, 1.0, size=n).astype(np_.float32)
        cos_theta = cos_half + (1.0 - cos_half) * u
        sin_theta = np_.sqrt(np_.clip(1.0 - cos_theta * cos_theta, 0.0, 1.0))
        phi = self._rng.uniform(0.0, 2.0 * math.pi, size=n).astype(np_.float32)
        local = np_.stack(
            [sin_theta * np_.cos(phi), sin_theta * np_.sin(phi), cos_theta],
            axis=1,
        ).astype(np_.float32)

        # local(+z up) → emit_direction 으로 회전 (Rodrigues)
        rotated = _rotate_z_to(local, direction, np_)

        speed_lo = self.config.initial_speed_min
        speed_hi = self.config.initial_speed_max
        speeds = self._rng.uniform(speed_lo, speed_hi, size=n).astype(np_.float32)
        self._vel[idx] = rotated * speeds[:, None]

        # lifetime: ±20% jitter
        life_mu = self.config.lifetime_sec
        life_jitter = self._rng.uniform(0.8, 1.2, size=n).astype(np_.float32)
        self._life[idx] = life_mu * life_jitter
        self._age[idx] = 0.0
        self._alive[idx] = True

    def _push_to_usd(self) -> None:
        """alive 배열을 PointInstancer 어트리뷰트로 전송."""
        np_ = self._np
        mask = self._alive
        n = int(mask.sum())

        UsdGeom = self._UsdGeom
        Gf = self._Gf
        Vt = self._Vt

        if n == 0:
            empty_i = Vt.IntArray([])
            empty_v = Vt.Vec3fArray([])
            empty_q = Vt.QuathArray([])
            self._instancer.GetProtoIndicesAttr().Set(empty_i)
            self._instancer.GetPositionsAttr().Set(empty_v)
            self._instancer.GetScalesAttr().Set(empty_v)
            self._instancer.GetOrientationsAttr().Set(empty_q)
            return

        positions = self._pos[mask]
        # lifetime 정규화 → fade-out scale 효과 (1 → 0.3)
        t_norm = np_.clip(self._age[mask] / np_.maximum(self._life[mask], 1e-6), 0.0, 1.0)
        scale_factor = (1.0 - 0.7 * t_norm).astype(np_.float32)

        proto_indices = Vt.IntArray([0] * n)

        pos_list = Vt.Vec3fArray([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))
                                  for p in positions])
        scale_list = Vt.Vec3fArray([Gf.Vec3f(float(s), float(s), float(s))
                                    for s in scale_factor])
        # orientation: identity quaternion (Quath: real, i, j, k)
        ident = Gf.Quath(1.0, 0.0, 0.0, 0.0)
        orient_list = Vt.QuathArray([ident] * n)

        self._instancer.GetProtoIndicesAttr().Set(proto_indices)
        self._instancer.GetPositionsAttr().Set(pos_list)
        self._instancer.GetScalesAttr().Set(scale_list)
        self._instancer.GetOrientationsAttr().Set(orient_list)


# =============================================================================
# 도우미 함수
# =============================================================================
def _rotate_z_to(local_vecs, target_dir, np_):
    """+z up 좌표계에서 만든 벡터들을 target_dir 방향으로 회전.

    Args:
        local_vecs: (n, 3) numpy array (assumed +z up cone samples)
        target_dir: (3,) unit vector — 원하는 분사 방향
        np_: numpy 모듈 참조

    Returns:
        (n, 3) 회전된 벡터들
    """
    z_axis = np_.array([0.0, 0.0, 1.0], dtype=np_.float32)
    t = target_dir.astype(np_.float32)
    # 회전축 = z × t, 각도 = arccos(z·t)
    cos_a = float(np_.clip(np_.dot(z_axis, t), -1.0, 1.0))
    if cos_a > 0.99999:
        return local_vecs.copy()
    if cos_a < -0.99999:
        # 180°: x축 기준으로 flip
        out = local_vecs.copy()
        out[:, 1] = -out[:, 1]
        out[:, 2] = -out[:, 2]
        return out
    axis = np_.cross(z_axis, t)
    axis = axis / np_.linalg.norm(axis)
    sin_a = float(np_.sqrt(max(0.0, 1.0 - cos_a * cos_a)))
    # Rodrigues rotation
    K = np_.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ], dtype=np_.float32)
    R = np_.eye(3, dtype=np_.float32) + sin_a * K + (1.0 - cos_a) * (K @ K)
    return (local_vecs @ R.T).astype(np_.float32)


__all__ = [
    "SparkEmitterConfig",
    "WeldingSparkEmitter",
    "add_sparks_pointinstancer",
]
