import argparse
import sys
from functools import partial
import math
import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt


dt = 0.1
def f(qt, dqt) :
    return qt + (dqt * dt)
    #return gtsam.Point2(qt[0] + dqt[0]*dt, qt[1]+dqt[1]*dt)


def distance(a, b):
     return math.sqrt(((a[0] - b[0]) ** 2) + ((a[1] - b[1]) ** 2))
#    return np.ndarray([abs(a[0]-b[0]),abs(a[1]-b[1])])
def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H:List[np.ndarray]):
    key_qt = this.keys()[0]
    key_dqt = this.keys()[1]
    key_qt1 = this.keys()[2]


    qt = v.atPoint2(key_qt)
    dqt = v.atPoint2(key_dqt)
    qt1 = v.atPoint2(key_qt1)

    if H is not None:
        H[0] = np.eye(2)  #derr/dqt
        H[1] = np.eye(2) * dt  #derr/ddqt
        H[2] = -np.eye(2)
    return np.array([abs(qt1[0]-f(qt,dqt)[0]), abs(qt1[1] - f(qt,dqt)[1])])


if __name__ == "__main__":
    #parse arguments
    parser = argparse.ArgumentParser(description='Trajectory Optimization')
    parser.add_argument("--start",nargs=2,type=float)
    parser.add_argument("--goal",nargs=2,type=float)
    parser.add_argument("--T",type=int)
    parser.add_argument("--x0",nargs=2,type=float)
    parser.add_argument("--x1",nargs=2,type=float)
    parser.parse_args(namespace=parser)

    st = parser.start
    gt = parser.goal
    x0 = parser.x0
    x1 = parser.x1
    start = np.array([st[0],st[1]])
    goal = np.array([gt[0],gt[1]])

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = parser.T

    sigma = 1
    noise_model = gtsam.noiseModel.Isotropic.Sigma(2,sigma)
    for t in range(0,T):
        kqt = gtsam.symbol("q",t)
        kdqt = gtsam.symbol("v",t)
        kqt1 = gtsam.symbol("q",t+1)

        keys = gtsam.KeyVector([kqt,kdqt,kqt1])
        factor = gtsam.CustomFactor(noise_model,keys,error_func)

        v.insert(kqt,np.array([0,0]))
        v.insert(kdqt,np.array([1,1]))
        graph.add(factor)

    #add start and goal factors
    prior = gtsam.noiseModel.Constrained.All(2)
    keyStart = gtsam.symbol("q",0)
    keyGoal = gtsam.symbol("q",T)
    v.insert(keyGoal,np.array([0,0]))
    graph.addPriorPoint2(keyStart,start,prior)
    graph.addPriorPoint2(keyGoal,goal,prior)

    #add extra constraints
    prior = gtsam.noiseModel.Constrained.All(2)
    cPoint1 = np.array([x0[0],x0[1]])
    cPoint2 = np.array([x1[0],x1[1]])
    keyC1 = gtsam.symbol("q",int(T/3))
    keyC2 = gtsam.symbol("q",int(2*T/3))
    graph.addPriorPoint2(keyC1,cPoint1,prior)
    graph.addPriorPoint2(keyC2,cPoint2,prior)

    # graph.print()
    # v.print()

    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, v,params).optimize()


    result.print()
    graph.printErrors(result)

    x = []
    y = []
    for t in range(T):
        kqt = gtsam.symbol("q",t)
        position = result.atPoint2(kqt)
        print(position)
        x.append(position[0])
        y.append(position[1])
    qtT = result.atPoint2(keyGoal)
    x.append(qtT[0])
    y.append(qtT[1])
    fix, ax = plt.subplots()

    plt.scatter(x,y)
    plt.plot(x,y)
    plt.show()






