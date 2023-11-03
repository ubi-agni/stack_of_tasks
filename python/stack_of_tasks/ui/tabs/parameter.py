from __future__ import annotations

from .base import Base


class Parameter(Base):
    def __init__(self) -> None:
        super().__init__()

    def add_action_callback(self):
        pass

    def remove_action_callback(self):
        pass
