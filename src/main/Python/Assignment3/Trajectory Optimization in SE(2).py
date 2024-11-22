import numpy as np
import math
import gtsam
from typing import List
dt = 0.1

def error_func(this: gtsam.CustomFactor, v:gtsam.Values,H:List[np.ndarray]):
    kq = this.keys()[0]
    kv = this.keys()[1]
    kt = this.keys()[2]

    q = v.atPose2(kq)
    v = v.atPose2(kv)
    t = v.atPose2(kt)

    prediction = q.

    if H is not None:
        H[0] = np.eye(3)
        H[1] = np.eye(3) * dt
        H[2] = -np.eye(3)

    return error

if __name__ == '__main__':
    start = gtsam.Pose2(0,0,0)
    goal = gtsam.Pose2(5,7,np.pi/2)

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 50

    noise_model = gtsam.noiseModel.Isotropic.Sigma(3,0)
    for t in range(T):
        kq = gtsam.Symbol("q",t)
        kv = gtsam.Symbol("v",t)
        kf = gtsam.Symbol("q",t+1)

        keys = gtsam.KeyVector([kq,kv,kf])
        factor = gtsam.CustomFactor(noise_model,keys,error_func)

        v.insert(kq,gtsam.Pose2(0,0,0))
        v.insert(kv, gtsam.Pose2(1,1,0.1))

        graph.add(factor)

    prior = gtsam.noiseModel.Constrained.All(1)

    key_start = gtsam.Symbol("q",0)
    key_goal = gtsam.Symbol("q",T)

    v.insert(key_goal,gtsam.Pose2(0,0,0))
    graph.addPriorPose2(key_goal,goal,prior)

    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, v,params).optimize()

    result.print()