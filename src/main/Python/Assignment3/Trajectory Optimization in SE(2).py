import numpy as np
import math
import gtsam
from typing import List

dt = 0.1


def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    kq = this.keys()[0]
    kv = this.keys()[1]
    kt = this.keys()[2]

    q = v.atPose2(kq)
    vel = v.atPose2(kv)
    t = v.atPose2(kt)

    prediction = [q.x() + dt * vel.x(), q.y() + dt * vel.y(), q.theta() + dt * vel.theta()]

    if H is not None:
        H[0] = np.eye(3)
        H[1] = np.eye(3) * dt
        H[2] = -np.eye(3)

    return np.array([prediction[0] - t.x(), prediction[1] - t.y(), prediction[2] - t.theta()])


if __name__ == '__main__':
    start = gtsam.Pose2(0, 0, 0)
    goal = gtsam.Pose2(5, 7, np.pi / 2)

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 50

    noise_model = gtsam.noiseModel.Isotropic.Sigma(3, 1)
    for t in range(T):
        kq = gtsam.symbol('q', t)
        kv = gtsam.symbol('v', t)
        kf = gtsam.symbol('q', t + 1)

        keys = gtsam.KeyVector([kq, kv, kf])
        factor = gtsam.CustomFactor(noise_model, keys, error_func)

        v.insert(kq, gtsam.Pose2(0, 0, 0))
        v.insert(kv, gtsam.Pose2(1, 1, 0.1))

        graph.add(factor)

    prior = gtsam.noiseModel.Constrained.All(3)

    key_start = gtsam.symbol('q', 0)
    key_goal = gtsam.symbol('q', T)

    v.insert(key_goal, gtsam.Pose2(0, 0, 0))
    graph.addPriorPose2(key_goal, goal, prior)
    graph.addPriorPose2(key_start, start, prior)

    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, v, params).optimize()

    result.print()
