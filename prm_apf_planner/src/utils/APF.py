import numpy as np
import matplotlib.pyplot as plt
import cv2


class APF:
    def __init__(self, influence_coefficient, repulsion_range):
        self.influence_coefficient = influence_coefficient
        self.repulsion_range = repulsion_range

    def repulsive_potential(self, x, y, ox, oy):
        id = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                id = i

        # distance between distance between current position q and obstacle q0
        dq = np.hypot(x - ox[id], y - oy[id])

        if dq <= self.repulsion_range:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self.influence_coefficient * (
                1.0 / dq - 1.0 / self.repulsion_range) ** 2
        else:
            return 0.0

    def sampling(self, map):
        sampling_points = 0
        obs_x, obs_y, white_reg, samples_obs_region, samples_open_region = [], [], [], [], []
        im_rep = map.copy()
        for i in range(im_rep.shape[0]):
            for j in range(im_rep.shape[1]):
                if im_rep[i][j] == 0:
                    obs_x.append(i)
                    obs_y.append(j)
                else:
                    white_reg.append([i, j])

        for i, j in enumerate(white_reg):
            rep = self.repulsive_potential(
                j[0], j[1], obs_x, obs_y)
            if rep > 1:
                im_rep[j[0], j[1]] = 127
                samples_obs_region.append(j)
                sampling_points = sampling_points + 1
            else:
                samples_open_region.append(j)
        return sampling_points, samples_obs_region, samples_open_region
