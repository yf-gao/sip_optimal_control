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


import numpy as np
import matplotlib.pyplot as plt

from py_sipoc_mr_utils import py_vel_profile_optimizer


def test():
    foo = py_vel_profile_optimizer.VelocityProfileOptimizer()
    foo.configureOptimizer(np.linspace(0.0, 3.0, 21))

    num_waypoints = 50
    waypoints = np.zeros((2*num_waypoints))
    for i in range(num_waypoints):
        waypoints[2*i] = 0.05*i
        waypoints[2*i+1] = 0.01*i

    foo.setRefPath(waypoints)
    robot_state = np.array([0., 0., 0., 0.8, 0.])
    x_ref = foo.computeReferenceTraj(robot_state)
    n_hrzn = len(x_ref) // 5 - 1

    fig = plt.figure(1)
    axes = fig.subplots(2,1)
    axes[0].plot(waypoints[0::2], waypoints[1::2], 'r--', linewidth=2, label="waypoints")
    axes[0].plot(x_ref[0::5], x_ref[1::5], 'b:', linewidth=2, label="ref traj")
    axes[0].legend()

    axes[1].plot(range(n_hrzn+1), x_ref[2::5], 'r--', linewidth=2, label="yaw")
    axes[1].plot(range(n_hrzn+1), x_ref[3::5], 'b:', linewidth=2, label="lin. v")
    axes[1].legend()
    plt.show()


if __name__ == "__main__":
    test()
