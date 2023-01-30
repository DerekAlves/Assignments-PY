from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import spatialmath as sm
from roboticstoolbox import *
from zmqRemoteApi import RemoteAPIClient
import time

# Tamanho dos braços
l1 = 0.475
l2 = 0.4

# Passo de integração

dt = 0.05

# Robot SerialLink

robot = DHRobot([
    RevoluteDH(a = l1, d = 0, alpha = 0),
    RevoluteDH(a = l2, d = 0, alpha = np.pi),
    PrismaticDH(a = 0, theta = 0, alpha = 0, qlim = [0, 0.1]),
    RevoluteDH(a = 0, d = 0, alpha = 0)
])

client = RemoteAPIClient()
sim = client.getObject('sim')

client.setStepping(True)
sim.startSimulation()

#Obtendo os handles das juntas no coppelia
r1 = sim.getObject('/MTB/axis')
r2 = sim.getObject('/MTB/link/axis')
p = sim.getObject('/MTB/link/axis/link/axis')
efetuador = sim.getObject('/MTB/suctionPad')
dummy = sim.getObject('/reference')


while True:
    q0 = sim.getJointPosition(r1)
    q1 = sim.getJointPosition(r2)
    q2 = sim.getJointPosition(p)

    q = [q0, q1, q2, 0]
    dum_pos = sim.getObjectPosition(dummy, -1)
    pad_pos = sim.getObjectPosition(efetuador, -1)

    pad_pose = robot.fkine(q)
    dummy_pose = sm.SE3(dum_pos[0], dum_pos[1], dum_pos[2])

    v, arrived = rtb.p_servo(pad_pose, dummy_pose, 1)

    jacob = robot.jacobe(q)
    jacob_i = np.linalg.pinv(jacob)

    q_dot = np.matmul(jacob_i, v)
    q += q_dot * dt

    error = np.linalg.norm(np.array(dum_pos) - np.array(pad_pos))
    sim.setJointPosition(r1, q[0])
    time.sleep(dt)
    sim.setJointPosition(r2, q[1])
    time.sleep(dt)
    sim.setJointPosition(p, q[2])
    time.sleep(dt)

    if error < 0.025:
        sim.stopSimulation()
        print('Robô alcançou o objetivo, simulação encerrada!\n')
        break