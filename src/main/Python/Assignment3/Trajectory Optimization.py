import sys
from functools import partial
import math
import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt

start = np.array([0,0])
goal = np.array([5,7])
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

    prediction = f(qt,dqt)
    error = distance(qt1,prediction)
    print("errors",error)
    if H is not None:
        H[0] = np.eye(1)  #derr/dqt
        H[1] = np.eye(1) * dt  #derr/ddqt
        H[2] = -np.eye(1)
    return np.array([error])


if __name__ == "__main__":
    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 50

    sigma = 1
    noise_model = gtsam.noiseModel.Isotropic.Sigma(1,sigma)
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






