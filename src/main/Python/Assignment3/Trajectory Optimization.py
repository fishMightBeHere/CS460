import sys
from functools import partial
import math
import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt

dt = 0.1
def f(dt, qt, dqt) :
    #ground truth function just is a straight line
    return qt + dqt * dt

# def cost constraint - cost function is distance from points


def error_func(y: np.ndarray, x: np.ndarray, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    key_qt = this.keys()[0]
    key_dqt = this.keys()[1]

    qt = v.atDouble(key_qt)
    dqt = v.atDouble(key_dqt)

    yPredict = qt + dqt * x
    error = yPredict -
    if H is not None:
        H[0] = np.eye(1)  #derr/dqt
        H[1] = np.eye(1) * x  #derr/ddqt
    return error


if __name__ == "__main__":
    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 100

    start = gtsam.Point2(0,0)
    goal = gtsam.Point2(100,100)


