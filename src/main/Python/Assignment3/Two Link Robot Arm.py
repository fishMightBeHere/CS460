import argparse

import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt

dt = 0.1
def f(q,v):
    return np.array([q[0] * gtsam.Rot2(v[0]*dt), q[1] * gtsam.Rot2(v[1]*dt)])

def error_func(this:gtsam.CustomFactor, v: gtsam.Values, H:List[np.ndarray]):
    key_Theta1 = this.keys()[0]
    key_Velocity1 = this.keys()[1]
    key_Theta2 = this.keys()[2]
    key_Velocity2 = this.keys()[3]
    key_Theta1Future = this.keys()[4]
    key_Theta2Future = this.keys()[5]


    t1 = v.atRot2(key_Theta1)

    v1 = v.atDouble(key_Velocity1)
    t2 = v.atRot2(key_Theta2)

    v2 = v.atDouble(key_Velocity2)
    t1f = v.atRot2(key_Theta1Future)
    t2f = v.atRot2(key_Theta2Future)

    prediction = f(np.array([t1, t2]), np.array([v1, v2]))
    if H is not None:
        H[0] = np.array([1,0])
        H[1] = np.array([dt,0])
        H[2] = np.array([0,1])
        H[3] = np.array([0,dt])
        H[4] = np.array([-1,0])
        H[5] = np.array([0,-1])

    return np.array([t1f.theta() - prediction[0].theta(),  t2f.theta() - prediction[1].theta()  ])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Two Link Robot Arm')
    parser.add_argument("--start",nargs=2,type=float)
    parser.add_argument("--goal",nargs=2,type=float)
    parser.add_argument("--T",type=int)
    parser.parse_args(namespace=parser)

    start = np.array([gtsam.Rot2(parser.start[0]),gtsam.Rot2(parser.start[1])])
    goal= np.array([gtsam.Rot2(parser.goal[0]),gtsam.Rot2(parser.goal[1])])

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = parser.T

    noise_model = gtsam.noiseModel.Isotropic.Sigma(2,1)
    for t in range(T):
        kTheta1 = gtsam.symbol("t",t)
        kVelocity1 = gtsam.symbol("v",t)
        kTheta2 = gtsam.symbol("u",t)
        kVelocity2 = gtsam.symbol("w",t)
        kTheta1Future = gtsam.symbol("t",t+1)
        kTheta2Future = gtsam.symbol("u",t+1)

        keys = gtsam.KeyVector([kTheta1,kVelocity1,kTheta2,kVelocity2,kTheta1Future,kTheta2Future])
        factor = gtsam.CustomFactor(noise_model,keys,error_func)

        v.insert(kTheta1,gtsam.Rot2(0.0))
        v.insert(kVelocity1,0.1)
        v.insert(kTheta2,gtsam.Rot2(0.0))
        v.insert(kVelocity2,0.1)

        graph.add(factor)

    #start and goal
    prior = gtsam.noiseModel.Constrained.All(1)
    keyStart1 = gtsam.symbol("t",0)
    keyStart2 = gtsam.symbol("u",0)
    keyGoal1 = gtsam.symbol("t",T)
    keyGoal2 = gtsam.symbol("u",T)
    v.insert(keyGoal1,gtsam.Rot2(0.0))
    v.insert(keyGoal2,gtsam.Rot2(0.0))
    graph.addPriorRot2(keyStart1,start[0],prior)
    graph.addPriorRot2(keyStart2,start[1],prior)
    graph.addPriorRot2(keyGoal1,goal[0],prior)
    graph.addPriorRot2(keyGoal2,goal[1],prior)

    params = gtsam.LevenbergMarquardtParams()
    #params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, v,params).optimize()
    #result.print()

    x = []
    y = []
    for t in range(T+1):
        keyTheta1 = gtsam.symbol("t",t)
        keyTheta2 = gtsam.symbol("u",t)
        x.append(result.atRot2(keyTheta1).theta())
        y.append(result.atRot2(keyTheta2).theta())
        print(result.atRot2(keyTheta1).theta(), result.atRot2(keyTheta2).theta())

    # fix, ax = plt.subplots()
    # plt.scatter(x,y)
    # plt.plot(x,y)
    # plt.show()