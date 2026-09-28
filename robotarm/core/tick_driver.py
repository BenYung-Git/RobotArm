"""Background tick driver — runs ``WebRobot.tick()`` at a fixed rate.

Extracted from ``web/app.py`` so tests can spin it up without spinning up
Flask. The same loop is used by both the web app and unit tests.

Usage:
    driver = TickDriver(robot, hz=60.0)
    driver.start()
    ...
    driver.stop()
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable

LOG = logging.getLogger("robotarm.tick")


class TickDriver:
    """Periodically call ``robot.tick()`` in a daemon thread.

    The driver is purely passive — it never *initiates* motion; it only
    advances interpolation when the robot already has joint targets.
    """

    def __init__(
        self,
        robot,
        *,
        hz: float = 60.0,
        on_tick: Callable[[], None] | None = None,
    ) -> None:
        self.robot = robot
        self.hz = float(hz)
        self.on_tick = on_tick
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="robotarm-tick"
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    def _loop(self) -> None:
        period = 1.0 / self.hz
        while not self._stop.is_set():
            with self._lock:
                try:
                    if self.robot.is_connected and not self.robot.is_e_stopped:
                        self.robot.tick()
                except Exception as e:  # pragma: no cover - defensive
                    LOG.warning("tick error: %s", e)
                if self.on_tick is not None:
                    try:
                        self.on_tick()
                    except Exception as e:
                        LOG.warning("on_tick hook error: %s", e)
            # Sleep with early-exit so stop() is responsive.
            self._stop.wait(period)
