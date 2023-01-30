from roboticstoolbox import *
import numpy as np
from numpy import *

np.set_printoptions(suppress=True)

def fKine(t1, t2, t3, d4):
    T = [transMatrix(0, 0.1, 0, 0)]
    T.append(transMatrix(t1, 0, 0.475, 0))
    T.append(transMatrix(t2, 0, 0.4, 0))
    T.append(transMatrix(t3, 0, 0, np.pi))
    T.append(transMatrix(0, d4, 0, 0))

    t = T[0]
    for i in range(1, len(T)):
        t = t @ T[i]
    return t

def transMatrix(t, d, a, al):
    return np.array([
        [np.cos(t), -1*np.sin(t)*np.cos(al), np.sin(t)*np.sin(al), a*np.cos(t)],
        [np.sin(t), np.cos(t)*np.cos(al), -1*np.cos(t)*np.sin(al), a*np.sin(t)],
        [0, np.sin(al), np.cos(al), d],
        [0, 0, 0, 1]
    ])

def ifkine(x, y, z, phi):
    l1 = 0.475
    l2 = 0.4

    if(l1 * l2 == 0):
        return "Erro - 1"
    t2 = arccos( (x ** 2 + y ** 2 - (l1 ** 2 + l2 ** 2)) / (2 * l1 * l2))

    if( x == 0 or (l1 + l2 * cos(t2)) == 0):
        return "Erro - 2"
    t1 = arctan2(y, x) - arctan2(l2 * sin(t2), l1 + l2 * cos(t2))
    
    t3 = phi - (t1 + t2)
    d4 = z
    if(not(0 <= d4 <= 0.1)):
        return "Erro - 3"
    return np.array([degrees(t1), degrees(t2), degrees(t3), d4])

robot = DHRobot([
        RevoluteDH(0.1, 0, 0),
        RevoluteDH(a=0.475, d=0, alpha=0),
        RevoluteDH(a=0.4, d=0, alpha=0),
        RevoluteDH(a=0, d=0, alpha=np.pi),
        PrismaticDH(a=0, theta=0, alpha=0, qlim=[0, 0.1])
    ])

print(robot)

#------------------------------------------------------------
#(a) theta_1 = 0; theta_2 = 0; theta_3 = 0; d_4 = 0
#print('FKine implementada:\n', fKine(0,0,0,0), '\n')
#print('FKine SerialLink:\n', robot.fkine([0, 0, 0, 0, 0]), '\n')

#(b) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4 = 0
#print('FKine implementada:\n', fKine(np.pi/2, -np.pi/2, 0, 0), '\n')
#print('FKine SerialLink:\n', robot.fkine([0, np.pi/2, -np.pi/2, 0, 0]), '\n')

#(c) theta_1 = pi/2; theta_2 = -pi/2; theta_3 = 0; d_4 = 0.05
print('FKine implementada:\n', fKine(np.pi/2, -np.pi/2, 0, 0.05), '\n')
print('FKine SerialLink:\n', robot.fkine([0, np.pi/2, -np.pi/2, 0, 0.05]), '\n')