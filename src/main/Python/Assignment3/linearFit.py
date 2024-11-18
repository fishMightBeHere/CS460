from functools import partial
import numpy as np
import gtsam
from typing import List, Optional

# "True" function with its respective parameters
def f(x, m=0.6, b = 1.5):
    return m * x + b

def error_func(y: np.ndarray, x: np.ndarray, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    """
    :param      y:     { Given data point at x: y = f(x) }
    :type       y:     { array of one element }
    :param      x:     { Value that produces y for some function f: y = f(x) }
    :type       x:     { Array of one element }
    :param      this:  The factor
    :type       this:  { CustomFactor }
    :param      v:     { Set of Values, accessed via a key  }
    :type       v:     { Values }
    :param      H:     { List of Jacobians: dErr/dInput. The inputs of THIS factor (the values)  }
    :type       H:     { List of matrices }
    """
    # First, get the keys associated to THIS factor. The keys are in the same order as when the factor is constructed
    key_m = this.keys()[0]
    key_b = this.keys()[1]

    # Access the values associated with each key. Useful function include: atDouble, atVector, atPose2, atPose3...
    m = v.atDouble(key_m)
    b = v.atDouble(key_b)

    # Compute the prediction (the function h(.))
    yp = m * x + b

    # Compute the error: H(.) - zi. Notice that zi here is "fixed" per factor
    error = yp - y

    # For comp. efficiency, only compute jacobians when requested
    if H is not None:
        # GTSAM always expects H[i] to be matrices. For this simple problem, each J is a 1x1 matrix
        H[0] = np.eye(1) * x # derr / dm
        H[1] = np.eye(1)     # derr / db
    return error


if __name__ == '__main__':

    graph = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    T = 100

    GT = [] # The ground truth, for comparison
    Z  = [] # GT + Normal(0, Sigma)

    # The initial guess values
    m = 1
    b = -1

    # Create the key associated to m
    km = gtsam.symbol('m', 0)
    kb = gtsam.symbol('b', 0)

    # Insert the initial guess of each key
    v.insert(km, m)
    v.insert(kb, b)

    # Create the \Sigma (a n x n matrix, here n=1)
    sigma = 1
    noise_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma)

    for i in range(T):
        GT.append(f(i))
        Z.append(f(i) + np.random.normal(0.0, sigma)) # Produce the noisy data

        # This are the keys associate to each factor.
        # Notice that for this simple example, the keys do not depend on T, but this may not always be the case
        keys = gtsam.KeyVector([km, kb])

        # Create the factor:
        #       Noise model - The Sigma associated to the factor
        #       Keys - The keys associated to the neighboring Variables of the factor
        #       Error function - The function that computes the error: h(.) - z
        #                        The function expected by CustomFactor has the signature
        #                           F(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray])
        #                         Because our function has more parameters (z and i), we need to *fix* this
        #                         which can be done via partial.
        gf = gtsam.CustomFactor(noise_model, keys,
                                partial(error_func, np.array([Z[i]]), np.array([i]) ))

        # add the factor to the graph.
        graph.add(gf)

    # Construct the optimizer and call with default parameters
    result = gtsam.LevenbergMarquardtOptimizer(graph, v).optimize()

    # We can print the graph, the values and evaluate the graph given some values:
    # result.print()
    # graph.print()
    # graph.printErrors(result)

    # Query the resulting values for m and b
    m = result.atDouble(km)
    b = result.atDouble(kb)
    print("m: ", m, " b: ", b)

    # Print the data for plotting.
    # Should be further tested that the resulting m, b actually fit the data
    for i in range(T):
        print(i, GT[i], Z[i])