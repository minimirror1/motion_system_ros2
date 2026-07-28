from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Action(Enum):
    HOME = 0
    STOP = 1
    MOVE = 2
    MOVE_TO_START = 3


@dataclass
class action_frame_t:
    robot_index: int
    action: Action