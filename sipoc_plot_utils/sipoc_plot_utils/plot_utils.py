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
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import matplotlib.transforms

def mat2ellip(mat):
    eig_val, eig_vec = np.linalg.eig(mat)
    if (eig_val < 0).any():
        print(eig_val)
        raise Exception("Matrix is not positive definite")
    large_idx = np.argmax(eig_val)
    large_vec = eig_vec[:, large_idx]
    small_idx = 1 - large_idx
    angle = np.arctan2(large_vec[1], large_vec[0])
    return np.sqrt(eig_val[large_idx]), np.sqrt(eig_val[small_idx]), np.degrees(angle)



def draw_capsule_arch_line_on_ax(ax: plt.Axes, state:np.ndarray, robot_cfg: tuple, color="tab:blue", plot_capsule_center=False, plot_capsule_center_line=False, linewidth=4, zorder=10, alpha=1.0):
    n_states = state.shape[0]
    dilation_radius = robot_cfg[0]
    half_length = robot_cfg[1]
    for idx_state in range(n_states):
        yaw_in_deg = state[idx_state, 2] / np.pi * 180
        vec = np.array([np.cos(state[idx_state, 2]), np.sin(state[idx_state, 2]) ]) * half_length
        perpendicular_vec = np.array([np.sin(state[idx_state, 2]), -np.cos(state[idx_state, 2]) ]) * dilation_radius

        center_line = np.vstack((state[[idx_state], :2] - vec[np.newaxis, :], state[[idx_state], :2] + vec[np.newaxis, :]))
        upper_line = np.vstack((state[[idx_state], :2] - vec[np.newaxis, :] + perpendicular_vec[np.newaxis, :],
                                state[[idx_state], :2] + vec[np.newaxis, :] + perpendicular_vec[np.newaxis, :]))
        ax.plot(upper_line[:, 0], upper_line[:, 1], color=color, linewidth=linewidth, zorder=zorder, alpha=alpha)
        lower_line = np.vstack((state[[idx_state], :2] - vec[np.newaxis, :] - perpendicular_vec[np.newaxis, :],
                                state[[idx_state], :2] + vec[np.newaxis, :] - perpendicular_vec[np.newaxis, :]))
        ax.plot(lower_line[:, 0], lower_line[:, 1], color=color, linewidth=linewidth, zorder=zorder, alpha=alpha)
        left_arch = patches.Arc((center_line[0, 0], center_line[0, 1]), 2*dilation_radius, 2*dilation_radius, angle=yaw_in_deg, theta1=90, theta2=270, color=color, linewidth=linewidth, alpha=alpha)
        right_arch = patches.Arc((center_line[1, 0], center_line[1, 1]), 2*dilation_radius, 2*dilation_radius, angle=yaw_in_deg, theta1=270, theta2=90, color=color, linewidth=linewidth, alpha=alpha)

        if plot_capsule_center_line:
            ax.plot(center_line[:, 0], center_line[:, 1], color=color, linewidth=linewidth, linestyle="dashed", dash_capstyle='round', dashes=(3, 2), zorder=zorder)
        if plot_capsule_center:
            ax.scatter(state[idx_state, 0], state[idx_state, 1], color=color, s=30, zorder=zorder)
        ax.add_patch(left_arch)
        ax.add_patch(right_arch)


def plot_dilated_polygon_on_axis(ax: plt.Axes, state:np.ndarray, polygon_vertices:np.ndarray, dilated_polygon_vertices:np.ndarray, color="tab:blue", plot_polygon:bool=False, linewidth=4, zorder=10, alpha=1.0):
    pc_list = []
    n_states = state.shape[0]
    for idx_state in range(n_states):
        padded_patch = patches.Polygon(dilated_polygon_vertices + state[[idx_state], :2], color=color, linewidth=linewidth, fill=False, zorder=zorder, alpha=alpha)
        mpl_transform = matplotlib.transforms.Affine2D().rotate_around(state[idx_state, 0], state[idx_state, 1], state[idx_state, 2])
        padded_patch.set_transform(mpl_transform)
        pc_list.append(padded_patch)

        if plot_polygon:
            polygon_patch = patches.Polygon(polygon_vertices + state[[idx_state], :2], color=color, linewidth=linewidth, fill=False, zorder=zorder, linestyle="dashed", alpha=alpha)
            polygon_patch.set_transform(mpl_transform)
            pc_list.append(polygon_patch)
    pc = PatchCollection(pc_list, match_original=True, zorder=zorder, linewidth=linewidth)
    ax.add_collection(pc)


def compute_dilated_polygon_vertices(polygon_vertices:np.ndarray, polygon_radius:float) -> np.ndarray:
    import alphashape
    radians = np.linspace(0, 2 * np.pi, 50)
    unit_circle = np.vstack((np.cos(radians), np.sin(radians))).T
    padded_polygon = []
    for vertex in polygon_vertices:
        for a in unit_circle:
            padded_polygon.append(vertex + polygon_radius * a)
    alpha_shape = alphashape.alphashape(padded_polygon, 0.)
    dilated_polygon_vertices = np.array(alpha_shape.exterior.coords)
    return dilated_polygon_vertices
