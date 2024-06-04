from __future__ import annotations

__all__ = ['FixedFrequencyLoopManager']


class FixedFrequencyLoopManager:
    def __init__(self, period_ns: int) -> None:
        ...

    def sleep(self) -> None:
        ...
