import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
import cv2


class Utils:
    def __init__(
            self,
            current,
            destination,
            occupancy_grid_map):
        self.current = np.array(current)
        self.destination = np.array(destination)
        self.occupancy_grid_map = occupancy_grid_map
        self.map_inflated = None

    def getInflatedMap(self):
        self.initializeMap()
        return self.map_inflated

    def drawMap(self):
        fig = plt.figure(figsize=(8, 8))
        currentAxis = plt.gca()
        curr = self.current
        dest = self.destination
        plt.scatter(curr[0], curr[1], s=200, c='green')
        plt.scatter(dest[0], dest[1], s=200, c='green')
        fig.canvas.draw()

    def initializeMap(self):
        thresh = 127
        im_bw = cv2.threshold(
            self.occupancy_grid_map,
            thresh,
            255,
            cv2.THRESH_BINARY)[1]
        kernel = np.ones((3, 3), np.uint8)
        im_bw = cv2.erode(im_bw, kernel, iterations=1)
        im_bw = cv2.copyMakeBorder(
            im_bw, 3, 3, 3, 3, cv2.BORDER_CONSTANT, value=0)
        self.map_inflated = cv2.rotate(im_bw, cv2.ROTATE_90_CLOCKWISE)
