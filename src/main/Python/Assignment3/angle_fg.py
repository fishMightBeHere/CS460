from functools import partial
import math
import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt

dt = 0.1
# "True" function with its respective parameters
def h(r, v):
    return r * gtsam.Rot2(v*dt)

def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    # First, get the keys associated to THIS factor. The keys are in the same order as when the factor is constructed
    key_r0 = this.keys()[0]
    key_v0 = this.keys()[1]
    key_r1 = this.keys()[2]

    # Access the values associated with each key. Useful function include: atDouble, atVector, atPose2, atPose3...
    r0 = v.atRot2(key_r0)
    v0 = v.atDouble(key_v0)
    r1 = v.atRot2(key_r1)

    # Compute the prediction (the function h(.))
    prediction = h(r0, v0)

    # Compute the error: H(.) - zi. Notice that zi here is "fixed" per factor
    # error = prediction.between(r1)
    error = r1.between(prediction)
    # r1.localCoordinates(r1)

    # For comp. efficiency, only compute jacobians when requested
    if H is not None: 
        # GTSAM always expects H[i] to be matrices. For this simple problem, each J is a 1x1 matrix
        H[0] = np.eye(1)
        H[1] = np.eye(1) * dt
        H[2] = -np.eye(1)

    return np.array([error.theta()])
    # return np.array([error])


if __name__ == '__main__':

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    
 
    # Create the \Sigma (a n x n matrix, here n=1)
    sigma = 1
    noise_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma)

    r0 = gtsam.Rot2(0.0)
    rT = gtsam.Rot2( 3* np.pi/2)
    T = 10
    # T = np.linspace(0, 1, num=10)
    for t in range(T):
        # Create the key associated to m
        kr0 = gtsam.symbol('r', t)    
        kv0 = gtsam.symbol('v', t)    
        kr1 = gtsam.symbol('r', t+1)    

        keys = gtsam.KeyVector([kr0, kv0, kr1])

        factor = gtsam.CustomFactor(noise_model, keys, error_func)
        
        # Insert the initial guess of each key 
        v.insert(kr0, gtsam.Rot2(0))
        v.insert(kv0, 0.1)

        # add the factor to the graph.
        graph.add(factor)

    prior = gtsam.noiseModel.Isotropic.Sigma(1, 1)
    kr0 = gtsam.symbol('r', 0)    
    krT = gtsam.symbol('r', T)    
    v.insert(krT, gtsam.Rot2(0))
    graph.addPriorRot2(kr0, r0, prior)
    graph.addPriorRot2(krT, rT, prior)

    # Construct the optimizer and call with default parameters
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, v,params).optimize()
    
    # We can print the graph, the values and evaluate the graph given some values:
    result.print()
    # graph.print()
    # graph.printErrors(result)

    x = []
    y = []
    p0 = np.array([[1.0],[0.0]])
    for t in range(T):
        kr0 = gtsam.symbol('r', t) 
        kv0 = gtsam.symbol('v', t) 
        r0 = result.atRot2(kr0)
        v0 = result.atDouble(kv0)
        # print(t, r0, v0)
        pt = r0.rotate(p0)
        x.append(pt[0])
        y.append(pt[1])
    rT = result.atRot2(krT)
    # print(t, rT)
    pt = rT.rotate(p0)
    x.append(pt[0])
    y.append(pt[1])

    fig, ax = plt.subplots()

    
    # plt.plot(x, y, label="X")
    plt.scatter(x, y, label="X")
    # plt.scatter(x, vel, label="Vel")
    plt.show()