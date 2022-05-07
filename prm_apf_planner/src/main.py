import numpy as np
import argparse
from utils import PRM, Utils, APF
import cv2
import matplotlib.pyplot as plt


def main():

    parser = argparse.ArgumentParser(
        description='PRM-Blended-Potential-Field-Path-Planning')
    parser.add_argument(
        '--start', nargs='+', type=int, default=[6, 1],
        help='start. Default: [2, 2]')
    parser.add_argument(
        '--goal', nargs='+', type=int, default=[19, 19],
        help='goal. Default: [19, 19]')
    parser.add_argument(
        '--FilePath', default='../map/map2.png',
        help='map file path')
    args = parser.parse_args()
    file_path = args.FilePath
    start = args.start
    goal = args.goal

    # Scale five times
    start[0] = start[0] * 5
    start[1] = start[1] * 5
    goal[0] = goal[0] * 5
    goal[1] = goal[1] * 5

    grid_map = cv2.imread(file_path, 0)
    utils = Utils(start, goal, grid_map)
    utils.drawMap()
    inflated_map = utils.getInflatedMap()
    influence_coefficient = 100
    repulsion_range = 5
    apf = APF(influence_coefficient, repulsion_range)
    sampling_points, samples_obs_region, samples_open_region = apf.sampling(
        inflated_map)
    Prm = PRM(
        start,
        goal,
        sampling_points,
        samples_obs_region,
        samples_open_region)
    Prm.prm()


if __name__ == '__main__':
    main()
