"""Small fixed-ROI presence check. Not a tracker or a proof of identity."""
import cv2
import numpy as np
from tower_live_hsv import masks


def body_present(rgb, boxes):
    support = []
    for c, (x0, y0, x1, y1, _) in boxes.items():
        hsv = cv2.cvtColor(np.ascontiguousarray(rgb[y0:y1, x0:x1]), cv2.COLOR_RGB2HSV)
        support.append(float(np.mean(masks(hsv)[c])))
    return sum(x >= .35 for x in support) >= 2
