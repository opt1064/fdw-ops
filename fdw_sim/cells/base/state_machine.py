"""셀 상태머신 (Cell State Machine).

FDW 분산지능 셀이 갖는 상태 전이 규칙을 정의한다.
   IDLE -> READY -> PROCESSING -> (READY|BLOCKED|FAULT) -> ...

전이가 허용되지 않을 경우 InvalidTransition 예외를 발생시킨다.
"""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Set
import logging

from fdw_sim.messaging.schemas import CellState

logger = logging.getLogger(__name__)


class InvalidTransition(RuntimeError):
    pass


# 허용된 상태 전이 그래프
_ALLOWED: Dict[CellState, Set[CellState]] = {
    CellState.OFFLINE:    {CellState.IDLE},
    CellState.IDLE:       {CellState.READY, CellState.FAULT, CellState.OFFLINE},
    CellState.READY:      {CellState.PROCESSING, CellState.BLOCKED, CellState.IDLE, CellState.FAULT},
    CellState.PROCESSING: {CellState.READY, CellState.BLOCKED, CellState.FAULT, CellState.IDLE},
    CellState.BLOCKED:    {CellState.READY, CellState.PROCESSING, CellState.FAULT, CellState.IDLE},
    CellState.FAULT:      {CellState.RECOVERY, CellState.OFFLINE},
    CellState.RECOVERY:   {CellState.IDLE, CellState.FAULT},
}


class CellStateMachine:
    """가벼운 finite-state machine.

    state 변경 시 등록된 콜백(on_enter / on_exit)을 호출한다.
    """

    def __init__(self, cell_id: str, initial: CellState = CellState.OFFLINE) -> None:
        self.cell_id = cell_id
        self._state = initial
        self._on_enter: Dict[CellState, List[Callable[[], None]]] = {}
        self._on_exit: Dict[CellState, List[Callable[[], None]]] = {}
        self._history: List[CellState] = [initial]

    # ------------------------------------------------------------------ API
    @property
    def state(self) -> CellState:
        return self._state

    @property
    def history(self) -> List[CellState]:
        return list(self._history)

    def can_transition(self, target: CellState) -> bool:
        return target in _ALLOWED.get(self._state, set())

    def transition(self, target: CellState, *, force: bool = False) -> None:
        if not force and not self.can_transition(target):
            raise InvalidTransition(
                f"[{self.cell_id}] {self._state.value} -> {target.value} is not allowed"
            )

        prev = self._state
        for cb in self._on_exit.get(prev, []):
            try:
                cb()
            except Exception:
                logger.exception("on_exit callback failed for %s", prev)

        self._state = target
        self._history.append(target)
        logger.debug("[%s] %s -> %s", self.cell_id, prev.value, target.value)

        for cb in self._on_enter.get(target, []):
            try:
                cb()
            except Exception:
                logger.exception("on_enter callback failed for %s", target)

    def on_enter(self, state: CellState, callback: Callable[[], None]) -> None:
        self._on_enter.setdefault(state, []).append(callback)

    def on_exit(self, state: CellState, callback: Callable[[], None]) -> None:
        self._on_exit.setdefault(state, []).append(callback)
