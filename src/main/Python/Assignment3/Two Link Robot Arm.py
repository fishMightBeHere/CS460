import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt

dt = 0.1
def f(q,v):
    return np.array([q[0] + (v[0]*dt), q[1] + (v[1]*dt)])

def error_func(this:gtsam.CustomFactor, v: gtsam.Values, H:List[np.ndarray]):
    key_Theta1 = this.keys()[0]
    key_Velocity1 = this.keys()[1]
    key_Theta2 = this.keys()[2]
    key_Velocity2 = this.keys()[3]
    key_Theta1Future = this.keys()[4]
    key_Theta2Future = this.keys()[5]


    t1 = v.atDouble(key_Theta1)

    v1 = v.atDouble(key_Velocity1)
    t2 = v.atDouble(key_Theta2)

    v2 = v.atDouble(key_Velocity2)
    t1f = v.atDouble(key_Theta1Future)
    t2f = v.atDouble(key_Theta2Future)

    prediction = f(np.array([t1, t2]), np.array([v1, v2]))
    if H is not None:
        H[0] = np.array([1,0])
        H[1] = np.array([dt,0])
        H[2] = np.array([0,1])
        H[3] = np.array([0,dt])
        H[4] = np.array([-1,0])
        H[5] = np.array([0,-1])

    return np.array([t1f - prediction[0],  t2f - prediction[1]  ])
    #return error
if __name__ == "__main__":

    start = np.array([2.5,0])
    goal= np.array([-2.5,0])

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 50

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

        v.insert(kTheta1,0.0)
        v.insert(kVelocity1,0.1)
        v.insert(kTheta2,0.0)
        v.insert(kVelocity2,0.1)

        graph.add(factor)

    #start and goal
    prior = gtsam.noiseModel.Constrained.All(1)
    keyStart1 = gtsam.symbol("t",0)
    keyStart2 = gtsam.symbol("u",0)
    keyGoal1 = gtsam.symbol("t",T)
    keyGoal2 = gtsam.symbol("u",T)
    v.insert(keyGoal1,0.0)
    v.insert(keyGoal2,0.0)
    graph.addPriorDouble(keyStart1,start[0],prior)
    graph.addPriorDouble(keyStart2,start[1],prior)
    graph.addPriorDouble(keyGoal1,goal[0],prior)
    graph.addPriorDouble(keyGoal2,goal[1],prior)

    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, v,params).optimize()
    result.print()

    x = []
    y = []
    for t in range(T+1):
        keyTheta1 = gtsam.symbol("t",t)
        keyTheta2 = gtsam.symbol("u",t)
        x.append(result.atDouble(keyTheta1))
        y.append(result.atDouble(keyTheta2))
        print(result.atDouble(keyTheta1), result.atDouble(keyTheta2))

    fix, ax = plt.subplots()
    plt.scatter(x,y)
    plt.plot(x,y)
    plt.show()