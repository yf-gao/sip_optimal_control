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
from sipoc_mr_support.configs import WorldConfig

class WorldMap:
    def __init__(self, world_cfg:WorldConfig) -> None:
        # 1: Free, 0: occupied
        self._world_cfg = world_cfg
        self.generate_occupancy_map()

    def generate_occupancy_map(self):
        grid_step = self._world_cfg.grid_step
        if self._world_cfg.world_type == "walkway":
            walkway_half_length = self._world_cfg.walkway_half_length
            walkway_half_width = self._world_cfg.walkway_half_width
            self._px_lsp = np.linspace(-walkway_half_length, walkway_half_length, int(2*walkway_half_length/grid_step), endpoint=False) + 0.5*grid_step
            self._py_lsp = np.linspace(-walkway_half_width, walkway_half_width, int(2*walkway_half_width/grid_step), endpoint=False) + 0.5*grid_step
            X, Y = np.meshgrid(self._px_lsp, self._py_lsp, indexing='ij')
            self._occupancy_map = np.ones_like(X)
            self._occupancy_map[Y >= 0.7*walkway_half_width] = 0
            self._occupancy_map[Y <= -0.7*walkway_half_width] = 0
            self._occupancy_map[np.logical_and(np.abs(X-0.4*walkway_half_length) <= 0.25*walkway_half_length, np.abs(Y+0.5*walkway_half_width) <= 0.3*walkway_half_width)] = 0
            self._occupancy_map[np.logical_and(np.abs(X+0.3*walkway_half_length) <= 0.25*walkway_half_length, np.abs(Y-0.4*walkway_half_width) <= 0.4*walkway_half_width)] = 0
        elif self._world_cfg.world_type == "docking_station":
            square_width = self._world_cfg.square_width
            self._px_lsp = np.linspace(-square_width, square_width, int(2*square_width/grid_step), endpoint=False) + 0.5*grid_step
            self._py_lsp = np.linspace(-square_width, square_width, int(2*square_width/grid_step), endpoint=False) + 0.5*grid_step
            X, Y = np.meshgrid(self._px_lsp, self._py_lsp, indexing='ij')
            self._occupancy_map = np.ones_like(X)
            self._occupancy_map[Y >=  0.9*square_width] = 0
            self._occupancy_map[Y <= -0.9*square_width] = 0
            self._occupancy_map[np.logical_and(X >= 0., Y >=  0.19*square_width)] = 0
            self._occupancy_map[np.logical_and(X >= 0., Y <= -0.19*square_width)] = 0
        elif self._world_cfg.world_type == "L_corridor":
            square_width = self._world_cfg.square_width
            self._px_lsp = np.linspace(-square_width, square_width, int(2*square_width/grid_step), endpoint=False) + 0.5*grid_step
            self._py_lsp = np.linspace(-square_width, square_width, int(2*square_width/grid_step), endpoint=False) + 0.5*grid_step
            X, Y = np.meshgrid(self._px_lsp, self._py_lsp, indexing='ij')
            self._occupancy_map = np.ones_like(X)
            self._occupancy_map[Y >= 0.55*square_width] = 0
            self._occupancy_map[X >= 0.55*square_width] = 0
            self._occupancy_map[np.logical_and(X <= 0.2*square_width, Y <= 0.2*square_width)] = 0
        elif self._world_cfg.world_type == "s_corridor":
            walkway_half_length = self._world_cfg.walkway_half_length
            walkway_half_width = self._world_cfg.walkway_half_width
            self._px_lsp = np.linspace(-walkway_half_length, walkway_half_length, int(2*walkway_half_length/grid_step), endpoint=False) + 0.5*grid_step
            self._py_lsp = np.linspace(-walkway_half_width, walkway_half_width, int(2*walkway_half_width/grid_step), endpoint=False) + 0.5*grid_step
            X, Y = np.meshgrid(self._px_lsp, self._py_lsp, indexing='ij')
            self._occupancy_map = np.ones_like(X)
            self._occupancy_map[Y >= 0.39*walkway_half_width + 0.5*walkway_half_width*np.sin(0.5*(X - 1.3))] = 0
            self._occupancy_map[Y <= -0.39*walkway_half_width + 0.5*walkway_half_width*np.sin(0.5*(X - 1.3))] = 0
        else:
            print("Map is not defined.")
            self._occupancy_map = None

    def plot_occupancy_map(self):
        from matplotlib import pyplot as plt
        plt.imshow(self._occupancy_map.T, cmap='gray', vmin=0, vmax=1, origin='lower')
        plt.show()

    def export_occupancy_map_to_img(self, filename):
        from PIL import Image, ImageOps
        img = ImageOps.flip(Image.fromarray((255*self._occupancy_map.T).astype(np.uint8)))
        img.save(filename)

    def export_occupancy_map_yaml(self, filename):
        map_origin = [self.px_lsp[0], self.py_lsp[0], 0.]
        with open(f"{filename}.yaml", "w") as yaml_file:
            yaml_file.write(f"image: {filename}.png\n")
            yaml_file.write(f"resolution: {self._world_cfg.grid_step}\n")
            yaml_file.write(f"origin: {map_origin}\n")
            yaml_file.write(f"negate: {0}\n")
            yaml_file.write(f"occupied_thresh: {0.65}\n")
            yaml_file.write(f"free_thresh: {0.2}\n")

    def export_occupancy_map_to_csv(self, filename):
        # NOTE: The occupancy notation in sdf (1: occupied, 0: free) is opposite to that in WorldMap
        # NOTE: The occupancy matrix is in the dimension of (height, width). So, we need to transpose it.
        np.savetxt(filename, 1 - self._occupancy_map.T, delimiter=",", fmt='%.1d', header = str(self._occupancy_map.shape[1]) + " " + str(self._occupancy_map.shape[0]), comments='# ')

    @property
    def occupancy_map(self):
        return self._occupancy_map

    @property
    def px_lsp(self):
        return self._px_lsp

    @property
    def py_lsp(self):
        return self._py_lsp

    @property
    def grid_step(self):
        return self._world_cfg.grid_step


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", help="The name of the world to generate.", type=str)
    parser.add_argument("--resolution", help="The resolution of the occupancy map.", type=float, default=0.02)
    args = parser.parse_args()
    if args.world is None:
        raise ValueError("Please specify the world type to generate.")
    print(f"Generating occupancy map for world type: {args.world} at resolution: {args.resolution}.")

    import os
    local_path = os.path.dirname(os.path.realpath(__file__))

    world_cfg = WorldConfig()
    world_cfg.world_type = args.world
    world_cfg.grid_step = args.resolution
    world_map = WorldMap(world_cfg)
    file_str = world_cfg.world_type
    print(f"lb_px_grid = {world_map._px_lsp[0]}, lb_py_grid = {world_map._py_lsp[0]}")
    print(f"grid_size = {world_cfg.grid_step}")
    print(f"map_height = {world_map._py_lsp.size}, map_width = {world_map._px_lsp.size}")
    world_map.plot_occupancy_map()
    world_map.export_occupancy_map_to_img(os.path.join(local_path, "maps", file_str + ".png"))
    world_map.export_occupancy_map_to_csv(os.path.join(local_path, "maps", file_str + "_occupancy_map.csv"))
