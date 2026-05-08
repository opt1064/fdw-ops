"""Message Bus 추상화.

초기 PoC에서는 In-Memory Pub/Sub으로 구현하고,
이후 ROS 2 / DDS / MQTT / ZeroMQ 백엔드로 교체 가능하도록 인터페이스를 분리.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Any, Callable, DefaultDict, List
import logging
import threading

logger = logging.getLogger(__name__)


class MessageBus(ABC):
    """모든 메시지 백엔드의 공통 인터페이스."""

    @abstractmethod
    def publish(self, topic: str, message: Any) -> None: ...

    @abstractmethod
    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None: ...

    @abstractmethod
    def shutdown(self) -> None: ...


class InMemoryBus(MessageBus):
    """단일 프로세스 내 Pub/Sub.

    PoC-1 검증용. ROS 2 Bridge로 교체할 때는 동일 인터페이스로 ROS2Bus를 작성.
    """

    def __init__(self) -> None:
        self._subscribers: DefaultDict[str, List[Callable[[Any], None]]] = defaultdict(list)
        self._lock = threading.RLock()
        self._published_count = 0

    def publish(self, topic: str, message: Any) -> None:
        with self._lock:
            callbacks = list(self._subscribers.get(topic, []))
            self._published_count += 1
        for cb in callbacks:
            try:
                cb(message)
            except Exception as e:
                logger.exception("Subscriber callback failed for topic '%s': %s", topic, e)

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            self._subscribers[topic].append(callback)
        logger.debug("Subscribed to '%s' (total subs: %d)", topic, len(self._subscribers[topic]))

    def shutdown(self) -> None:
        with self._lock:
            self._subscribers.clear()
        logger.info("InMemoryBus shutdown. Total messages published: %d", self._published_count)


# =============================================================================
# Standard Topic Names (FDW-OPS 컨벤션)
# =============================================================================
class Topics:
    CELL_STATUS = "/fdw/cell_status"
    CELL_EVENT = "/fdw/cell_event"
    DISPATCH_COMMAND = "/fdw/dispatch_command"
    MATERIAL_TRANSFER = "/fdw/material_transfer"
    KPI_LOG = "/fdw/kpi_log"
    JOB_CREATED = "/fdw/job_created"
    JOB_COMPLETED = "/fdw/job_completed"
    FAULT = "/fdw/fault"
