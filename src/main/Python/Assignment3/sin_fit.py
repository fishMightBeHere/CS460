from functools import partial
import math
import numpy as np
import gtsam
from typing import List, Optional
import matplotlib.pyplot as plt

# "True" function with its respective parameters
def h(t, a, b):
    return a * np.sin(b * t)

def error_func(z: np.ndarray, t: np.ndarray, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    # First, get the keys associated to THIS factor. The keys are in the same order as when the factor is constructed
    key_a = this.keys()[0]
    key_b = this.keys()[1]

    # Access the values associated with each key. Useful function include: atDouble, atVector, atPose2, atPose3...
    a = v.atDouble(key_a)
    b = v.atDouble(key_b)

    # Compute the prediction (the function h(.))
    prediction = h(t,a,b)

    # Compute the error: H(.) - zi. Notice that zi here is "fixed" per factor
    error = prediction - z

    # For comp. efficiency, only compute jacobians when requested
    if H is not None: 
        # GTSAM always expects H[i] to be matrices. For this simple problem, each J is a 1x1 matrix
        H[0] = np.eye(1) * math.sin( b * t )
        H[1] = np.eye(1) * a * t * math.cos( b * t )
    return error


if __name__ == '__main__':

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    
    GT = [] # The ground truth, for comparison
    Z  = [] # GT + Normal(0, Sigma)
    
    # The initial guess values
    a = 1; 
    b = 1; 
    
    # Create the key associated to m
    ka = gtsam.symbol('a', 0)    
    kb = gtsam.symbol('b', 0)    

    # Insert the initial guess of each key 
    v.insert(ka, a)
    v.insert(kb, b)

    # Create the \Sigma (a n x n matrix, here n=1)
    sigma = 1
    noise_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma)

    T = np.linspace(-5, 5, num=100)
    for t in T:
        gt = h(t, 2, 0.5)
        z = gt + np.random.normal(0.0, sigma)
        GT.append(gt)
        Z.append(z) # Produce the noisy data
        
        keys = gtsam.KeyVector([ka, kb])

        factor = gtsam.CustomFactor(noise_model, keys,
                                partial(error_func, np.array([z]), np.array([t]) ))
        
        # add the factor to the graph.
        graph.add(factor)

    # Construct the optimizer and call with default parameters
    result = gtsam.LevenbergMarquardtOptimizer(graph, v).optimize()
    
    # We can print the graph, the values and evaluate the graph given some values:
    # result.print()
    # graph.print()
    # graph.printErrors(result)

    # Query the resulting values for m and b
    a = result.atDouble(ka)
    b = result.atDouble(kb)
    print("a: ", a, " b: ", b)


    # plt.rc('FG', linewidth=2.5)
    fig, ax = plt.subplots()

    plt_GT = ax.plot(T, GT, label='GT')
    plt.scatter(T, Z, label="Z")

    Xp = h(T, a, b)
    plt_GT = ax.plot(T, Xp, label='FG')

    plt.show()