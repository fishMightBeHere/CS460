import sys
from functools import partial
import math
import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt


def f(x, a=0.045, b=0.2, c=0.7, d=4.86):
    return a * x * x * x + b * x * x + c * x + d


def error_func(y: np.ndarray, x: np.ndarray, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    key_a = this.keys()[0]
    key_b = this.keys()[1]
    key_c = this.keys()[2]
    key_d = this.keys()[3]

    a = v.atDouble(key_a)
    b = v.atDouble(key_b)
    c = v.atDouble(key_c)
    d = v.atDouble(key_d)

    yPredct = a * x * x * x + b * x * x + c * x + d
    error = yPredct - y

    if H is not None:
        H[0] = np.eye(1) * x * x * x  # derr/da
        H[1] = np.eye(1) * x * x  # derr/db
        H[2] = np.eye(1) * x  # derr/dc
        H[3] = np.eye(1)  # derr/dd
    return error


if __name__ == '__main__':
    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 100

    GT = []
    Z = []

    a = float(sys.argv[1])
    b = float(sys.argv[2])
    c = float(sys.argv[3])
    d = float(sys.argv[4])

    ka = gtsam.symbol('a', 0)
    kb = gtsam.symbol('b', 0)
    kc = gtsam.symbol('c', 0)
    kd = gtsam.symbol('d', 0)

    v.insert(ka, a)
    v.insert(kb, b)
    v.insert(kc, c)
    v.insert(kd, d)

    sigma = 1
    noise_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma)
    for i in range(T):
        GT.append(f(i))
        Z.append(f(i) + np.random.normal(0.0, sigma))

        keys = gtsam.KeyVector([ka, kb, kc, kd])

        gf = gtsam.CustomFactor(noise_model, keys, partial(error_func, np.array([Z[i]]), np.array([i])))

        graph.add(gf)

    result = gtsam.LevenbergMarquardtOptimizer(graph,v).optimize()

    a = result.atDouble(ka)
    b = result.atDouble(kb)
    c = result.atDouble(kc)
    d = result.atDouble(kd)

    print(" a:",a," b:",b," c:",c," d:",d)
    fix, ax = plt.subplots()

    plt_GT = ax.plot(range(T),GT,label="GT")
    plt.scatter(range(T),Z,label="Z")

    Xp = f(np.linspace(0,100,num=T),a,b,c,d)
    plt_GT = ax.plot(range(T),Xp,label="FG")

    plt.show()

