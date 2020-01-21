from mars_interface import *
import math
import intersect
import numpy as np
import numpy.matlib
import time

population = None
numP = 0
pSize = 0


def init(numParticles, particleSize, minState, maxState):
    global population, numP, pSize
    numP = numParticles
    pSize = particleSize
    population = np.zeros((numP, pSize+1))
    w = 1. / numP
    minState = np.append(minState, [w])
    maxState = np.append(maxState, w)
    r = np.matlib.rand(numP, pSize+1)
    diff = np.subtract(maxState, minState)
    out = diff
    out2 = minState
    for i in range(numP-1):
        out = np.vstack((out, diff))
        out2 = np.vstack((out2, minState))
    diff = out
    minState = out2
    np.multiply(diff, r, population)
    np.add(population, minState, population)


def normalize_weights():
    global population, numP, pSize
    scale = 1/np.sum(population[:,pSize])
    np.multiply(population[:,pSize], scale, population[:,pSize])


def apply_motion_model_(wheels):
    global population, numP, pSize
    # dt = 0.04
    # length of axel connection b = 0.485584
    # dt/b = 0.082375
    c = wheels[1]-wheels[0];
    # 0.9 as additional damping factor for rotations
    a = 0.9*c*0.082375
    dResult = np.add(population[:,2], a)
    for p in dResult:
       if p > math.pi:
           p -= 2.*math.pi
       elif p < -math.pi:
           p += 2.*math.pi

    g = 0.5*(wheels[0]+wheels[1])*0.04
    cos = np.cos(dResult)*g
    sin = np.sin(dResult)*g
    np.add(population[:,0], cos, population[:,0])
    np.add(population[:,1], sin, population[:,1])
    np.add(population[:,2], a, population[:,2])
    r = np.random.rand(numP,pSize+1)
    np.subtract(r, 0.5, r)
    np.multiply(r, 0.05+0.2*math.sqrt(population[0][pSize]), r)
    np.multiply(r[0], 0.0, r[0]);
    np.multiply(r[:,pSize], 0.0, r[:,pSize])
    np.add(population, r, population)


def generate_measurement(particle, walls, debug=False, color=1):
    m = []
    d = particle[2]
    cos = math.cos(d)
    sin = math.sin(d)
    r = np.array([[cos, -sin],
                  [sin, cos]])

    # Laser scanner positions fl, fr, bl, br
    basePos = np.array([[ 0.320234,  0.230233],
                        [ 0.320234, -0.230234],
                        [-0.306395,  0.214315],
                        [-0.306395, -0.214316]])
    directions = np.array([0.262, -1.309, 1.833, -2.879])
    np.add(directions, d, directions)
    for i in range(4):
        p = np.array(basePos[i])
        p = np.dot(r, p)
        p[0] += particle[0]
        p[1] += particle[1]
        rOffset = 0.
        for l in range(4):
            p2 = np.array([math.cos(directions[i]+rOffset),
                           math.sin(directions[i]+rOffset)])
            np.multiply(p2, 4., p2)
            np.add(p, p2, p2)
            line = np.append(p, p2)
            l1 = intersect.get_intersect(line[0], line[1], line[2], line[3])
            l1 = np.min([l1, 1.0])
            if debug: 
                np.subtract(p2, p, p2)
                np.multiply(p2, l1, p2)
                np.add(p2, p, p2)
                if color == 1:
                    appendLines("debugRays", p[0], p[1], 0.35)
                    appendLines("debugRays", p2[0], p2[1], 0.35)
                else:
                    appendLines("debugRays2", p[0], p[1], 0.35)
                    appendLines("debugRays2", p2[0], p2[1], 0.35)
            m.append(l1*4.)
            rOffset += 0.349
    return np.array(m)


def sample_and_weight(walls, realMeasure, wheels):
    global population, numP, pSize
    #start= time.clock()
    sigma = 1./(2.*10*10)
    apply_motion_model_(wheels)

    for particle in population:
        vMeasure = generate_measurement(particle, walls)
        s = np.sum(np.square(np.subtract(vMeasure, realMeasure)))
        particle[pSize] = s # math.exp(s*sigma)
    #print(str((time.clock()-start)))


def sort_particles():
    global population, pSize
    population = population[population[:,pSize].argsort()]
    #logMessage("da:")
    #for i in population:
    #    logMessage(str(i))


def select_best_trunc():
    global population, pSize, numP
    sort_particles()
    i = int(math.floor(numP / 5))
    p = np.array(population[:i])
    for i in range(4):
        p = np.append(p, population[:i], 0)
    l = len(population)-len(p)
    for i in range(l):
        p = np.append(p, population[i:i+1], 0)
    population = p
