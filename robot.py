"""Robot ABC — common interface for all robots."""

from abc import ABC, abstractmethod
import numpy as np


class Robot(ABC):
    @abstractmethod
    def step(self, mocap_targets: dict) -> None:
        """Apply mocap targets and advance simulation one step.

        Args:
            mocap_targets: {body_name: (pos[3], quat[4])}
        """

    @abstractmethod
    def render(self) -> np.ndarray:
        """Render current scene to an RGB image (H, W, 3) uint8."""

    @abstractmethod
    def get_joint_angles(self) -> dict:
        """Return {hardware_joint_name: angle_in_degrees}."""
