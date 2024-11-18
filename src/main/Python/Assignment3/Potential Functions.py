import math
import sys
from typing import Tuple, List
import numpy as np

def delUAttract(q: Tuple[float, float, float], a: int, goal: Tuple[float, float, float]):
    # attraction function
    # delUatt(x) = -a(x-xGoal)
    return np.array([10*(-a * (q[0] - goal[0]))/euclidian(q,goal), 10*(-a * (q[1] - goal[1]))/euclidian(q,goal),10* (-a * (q[2] - goal[2]))/euclidian(q,goal)])


def uRepulse(q: Tuple[float, float, float], env: list[list[float]], k):
    # repulsion equation
    # delRepel = -k(1/euclidian - 1/d*) ( x-obstacleX)/euclidean^3
    if len(env) == 0:
        return np.array([0,0,0])

    closest_obstacle = min(env, key=lambda obj: euclidian((obj[0], obj[1], obj[2]), q))

    frame = (closest_obstacle[0], closest_obstacle[1], closest_obstacle[2])
    r = max([closest_obstacle[3], closest_obstacle[4]]) / 2.0
    if euclidian(q, frame) < 5 * r:
        return np.array([
            -k * (1 / euclidian(q,frame) - 1 / (5*r)) * ((q[0]-closest_obstacle[0])/(math.pow(euclidian(q, frame), 3))),
            -k * (1 / euclidian(q,frame) - 1 / (5*r)) * ((q[1]-closest_obstacle[1])/(math.pow(euclidian(q,frame),3))),
            0
            #-k * (1 / euclidian(q,frame) - 1 / (2*r)) * ((q[2]-closest_obstacle[2])/(math.pow(euclidian(q,frame),3)))
        ])
    return np.array([0,0,0])


def euclidian(a: Tuple[float, float, float], b: Tuple[float, float, float]):
    return math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))


def delU(q: Tuple[float, float, float], goal: Tuple[float, float, float],env: list[list[float]]):
    return delUAttract(q,2,goal) - uRepulse(q,env, 40000)


def potential(start: np.array, goal:np.array, env: list[list[float]]):
    path: list[np.array] = [np.array(start)]
    cur = path[0]
    while euclidian(cur,goal) > 1:
        path.append(cur + 0.001*delU(cur,goal,env))
        cur = path[-1]
    return path

def loadEnviroment(file):
    f = open(file, "r")
    obstacles: list[list[float]] = []
    for line in f:
        obstacles.append(list(map(float, line.split(','))))
    return obstacles

def printPath(path: list[list[float]]):
    for p in path:
        print(p[0],p[1],p[2])


if __name__ == '__main__':
    print(sys.argv)
    env = loadEnviroment(sys.argv[1])
    start  = np.array(list(map(float,sys.argv[2].split(";"))))
    goal = np.array(list(map(float,sys.argv[3].split(";"))))
    output = potential(start,goal,env)
    printPath(output)

