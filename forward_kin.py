import numpy as np
from math import sin, cos
import math
import sys

degrees = False
PI = math.pi
np.set_printoptions(precision=3, suppress=True)

def toRad(x):
    return x / 180.0 * PI

def rotX(theta):
    theta = toRad(theta) if degrees else theta
    s = sin(theta)
    c = cos(theta)
    return np.matrix([[1, 0, 0 ],
                      [0, c, -s],
                      [0, s, c ]])


def rotX_homog(theta):
    theta = toRad(theta) if degrees else theta
    s = sin(theta)
    c = cos(theta)
    return np.matrix([[1, 0, 0 , 0],
                      [0, c, -s, 0],
                      [0, s, c , 0],
                      [0, 0, 0 , 1]])


def rotY(theta):
    theta = toRad(theta) if degrees else theta
    s = sin(theta)
    c = cos(theta)
    return np.matrix([[c,  0, s],
                      [0,  1, 0], 
                      [-s, 0, c]])


def rotY_homog(theta):
    theta = toRad(theta) if degrees else theta
    s = sin(theta)
    c = cos(theta)
    return np.matrix([[c,  0, s, 0],
                      [0,  1, 0, 0], 
                      [-s, 0, c, 0],
                      [0,  0, 0, 1]])

def rotZ(theta):
    theta = toRad(theta) if degrees else theta
    s = sin(theta)
    c = cos(theta)
    return np.matrix([[c, -s, 0],
                      [s,  c, 0],
                      [0,  0, 1]])

def rotZ_homog(theta):
    theta = toRad(theta) if degrees else theta
    s = sin(theta)
    c = cos(theta)
    return np.matrix([[c, -s, 0, 0],
                      [s,  c, 0, 0],
                      [0,  0, 1, 0],
                      [0,  0, 0, 1]])
def trans(x, y, z):
    return np.matrix([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1]])

def to_list(m):
    m1 = m.tolist()
    for i in range(0, len(m1)):
        for j in range(0, len(m1[i])):
            m1[i][j] = 0 if abs(m1[i][j]) < 0.000001 else m1[i][j] 
    return m1

def unbox_V(T):
    return np.matrix([T[2,1], T[0,2], T[1,0], T[0,3], T[1,3], T[2,3]]).T

def vect_3(x, y, z):
    return np.matrix([[x], [y], [z]])

def skew_sym(v):
    x = v[0,0]
    y = v[1,0]
    z = v[2,0]

    return np.matrix([[0, -z, y], [z, 0, -x], [-y, x, 0]])

def box_V(T):
    w = T[0:3,0]
    p = T[3:6,0]
    wp = np.concatenate((skew_sym(w),p), axis=1)
    return np.concatenate((wp, np.matrix([0, 0, 0, 0])), axis=0)

from  scipy.linalg import expm

# measurements from robot
m1 = 0.1442
m2 = 0.2454
m3 = 0.4004
m4 = 0.5352
m5 = 0.6316
m6 = 0.7288


S = [
        (np.matrix([0, 0, 0, 1, 0, 0])).T,
        (np.matrix([0, 0, 0, 0, 1, 0])).T,
        (np.matrix([0, 0, 1, 0, 0, 0])).T,
        (np.matrix([1, 0, 0, 0, m2, 0])).T,
        (np.matrix([1, 0, 0, 0, m3, 0])).T,
        (np.matrix([1, 0, 0, 0, m4, 0])).T,
        (np.matrix([0, 0, 1, 0, m5, 0])).T
]

M = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, m6], [0, 0, 0, 1]])

I = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

# d1, d2, theta1 - theta5
inputs = [0, 0, 0, PI/8.0, -PI/4.0, -PI/4.0, 0]

for i in range(len(S)):
    st = inputs[i] * box_V(S[i])
    ex = expm(st)
    I = I * ex

out = I * M


print(out)
