# Copyright (C) 2025 Robert Bosch GmbH

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# -------------------------------------
# Author: Yunfan Gao
#


from dataclasses import dataclass, field
import numpy as np

@dataclass
class RobotConfig:
    min_forward_velocity:     float = field(default = 0.0)
    max_forward_velocity:     float = field(default = 3.0)
    max_angular_velocity:     float = field(default = 1.0)
    min_forward_acceleration: float = field(default = -6.0)
    max_forward_acceleration: float = field(default = 3.0)
    max_angular_acceleration: float = field(default = 1.0)
    max_forward_velocity_e:   float = field(default = 1.0)
    max_angular_velocity_e:   float = field(default = 1.0)
    robot_capsule_radius:     float = field(default = 0.4)
    robot_capsule_half_length: float = field(default = 1.5)
    robot_polygon_vertices_flattened: np.ndarray = field(default_factory=lambda: np.array([]))
    robot_polygon_radius:      float = field(default = 0.1)


@dataclass
class WorldConfig:
    world_type:          str = field(default="walkway")
    grid_step:           float = field(default=0.02)
    walkway_half_length: float = field(default=8.0)
    walkway_half_width:  float = field(default=2.0)
    square_width:        float = field(default=3.0)


@dataclass
class SipocSolverConfig:
    nx:      int = field(default=5)
    nu:      int = field(default=2)
    n_hrzn:  int = field(default=20)
    delta_t:              float = field(default=0.1)
    shooting_nodes: np.ndarray = field(default_factory=lambda: np.array([]))

    Q_position:             float = field(default=1e-1)
    Q_heading:              float = field(default=1e-3)
    Q_forward_velocity:     float = field(default=1e-1)
    Q_angular_velocity:     float = field(default=1e-5)
    R_forward_acceleration: float = field(default=1e-3)
    R_angular_acceleration: float = field(default=1e-3)

    slack_l1_penalty: float = field(default=1e3)
    slack_l2_penalty: float = field(default=1e2)

    def __post_init__(self):
        self.Q = np.diag([self.Q_position, self.Q_position, self.Q_heading, self.Q_forward_velocity, self.Q_angular_velocity])
        self.R = np.diag([self.R_forward_acceleration, self.R_angular_acceleration])
