from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import *
from zmqRemoteApi import RemoteAPIClient
import time

#parametros
#j1
d1 = 0.1103
a1 = 0
al1 = -np.pi/2
off1 = 0

#j2
d2 = 0
a2 = 0.125
al2 = 0 
off2 = -np.pi/2

#j3
d3 = 0
a3 = 0.096
al3 = 0
off3 = np.pi/2

#j4
d4 = 0
a4 = -0.0275
al4 = np.pi/2
off4 = np.pi/2

#j5
d5 = 0.065
a5 = 0
al5 = 0
off5 = 0

robot = DHRobot([
        RevoluteDH(a=a1, d=d1, alpha=al1, offset=off1),
        RevoluteDH(a=a2, d=d2, alpha=al2, offset=off2),
        RevoluteDH(a=a3, d=d3, alpha=al3, offset=off3),
        RevoluteDH(a=a4, d=d4, alpha=al4, offset=off4),
        RevoluteDH(a=a5, d=d5, alpha=al5, offset=off5)     
    ])

print(robot)


def transMatrix(t, d, a, al, off=0.):
    t = t + off
    cost = np.cos(t)
    sent = np.sin(t)
    cosa = np.cos(al)
    sena = np.sin(al)

    return np.array([
        cost, -sent*cosa, sent*sena, a*cost,
        sent, cost*cosa, -cost*sena, a*sent,
        0, sena, cosa, d,
        0, 0, 0, 1], dtype=float).reshape(4,4)

def jointsTransforms(t1, t2, t3, t4, t5):
    return  np.array([transMatrix(t1, d1, a1, al1, off1), 
            transMatrix(t2, d2, a2, al2, off2), 
            transMatrix(t3, d3, a3, al3, off3), 
            transMatrix(t4, d4, a4, al4, off4), 
            transMatrix(t5, d5, a5, al5, off5)])

def fKine(joints):
    f = joints[0]
    for i in range(1, len(joints)):
        f = f @ joints[i]
    return f

def Jacob(t1, t2, t3, t4, t5):

    trans = jointsTransforms(t1, t2, t3, t4, t5)

    p1 = trans[0]
    p2 = p1 @ trans[1]
    p3 = p2 @ trans[2]
    p4 = p3 @ trans[3]
    p5 = p4 @ trans[4]
    pe = p5[0:3, -1]

    jo1 = np.array([0, 0, 1]).T
    jp1 = np.cross(jo1, pe)
    jo2 = p1[0:3, 2]
    jp2 = np.cross(jo2, pe - p1[0:3, -1])
    jo3 = p2[0:3, 2]
    jp3 = np.cross(jo3, pe - p2[0:3, -1])
    jo4 = p3[0:3, 2]
    jp4 = np.cross(jo4, pe - p3[0:3, -1])
    jo5 = p4[0:3, 2]
    jp5 = np.cross(jo5, pe - p5[0:3, -1])
   
    return np.array([   np.concatenate([jp1, jo1]), 
                        np.concatenate([jp2, jo2]), 
                        np.concatenate([jp3, jo3]), 
                        np.concatenate([jp4, jo4]),
                        np.concatenate([jp5, jo5])]).T

def pos2pose(pos):
    x, y, z = pos[0:3, -1]

    for i in range(3):
        for j in range(3):
            if (np.abs(pos[i,j]) < 0.01):
                pos[i,j] = 0.0

    oz = np.arctan2( pos[1, 0], pos[0, 0])
    oy = np.arctan2( -pos[2, 0], np.sqrt( pos[2, 1] ** 2 + pos[2, 2] ** 2))
    ox = np.arctan2( pos[2, 1], pos[2, 2])

    return np.array([x, y, z, ox, oy, oz]).reshape(6, 1)

def jointPos():
    return np.array([   sim.getJointPosition(r1), 
                        sim.getJointPosition(r2), 
                        sim.getJointPosition(r3), 
                        sim.getJointPosition(r4), 
                        sim.getJointPosition(r5)])

def setJointPos(q):
    sim.setJointTargetPosition(r1, q[0])
    sim.setJointTargetPosition(r2, q[1])
    sim.setJointTargetPosition(r3, q[2])
    sim.setJointTargetPosition(r4, q[3])
    sim.setJointTargetPosition(r5, q[4])

# Passo de integração
dt = 0.05

client = RemoteAPIClient()
sim = client.getObject('sim')

client.setStepping(True)
sim.startSimulation()

#Obtendo os handles das juntas no coppelia
r1 = sim.getObject('/world_visual/ref/base_link_respondable/theta1')
r2 = sim.getObject('/world_visual/ref/base_link_respondable/theta1/virtual1_respondable/theta2')
r3 = sim.getObject('/world_visual/ref/base_link_respondable/theta1/virtual1_respondable/theta2/l1_respondable/theta3')
r4 = sim.getObject('/world_visual/ref/base_link_respondable/theta1/virtual1_respondable/theta2/l1_respondable/theta3/l2_respondable/theta4')
r5 = sim.getObject('/world_visual/ref/base_link_respondable/theta1/virtual1_respondable/theta2/l1_respondable/theta3/l2_respondable/theta4/l3_respondable/v_joint1/virtual2_visual/theta5')

efetuador = sim.getObject('/world_visual/ref/base_link_respondable/theta1/virtual1_respondable/theta2/l1_respondable/theta3/l2_respondable/theta4/l3_respondable/v_joint1/virtual2_visual/theta5/end_effector_respondable')

dummy = sim.getObject('/reference')

setJointPos((0,0,0,0,0))

j = jointsTransforms(0,0,0,0,0)
print(j)
print('Fkine:---------------\n', fKine(j), '\n------------------------\n')
eff_pose = pos2pose(fKine(j))

man = []
effpose_list = []
joints_list = []

while True:

    dum_pos = np.array(sim.getObjectPosition(dummy, sim.handle_world) + sim.getObjectOrientation(dummy, sim.handle_world)).reshape(6, 1)
    pad_pos = sim.getObjectPosition(efetuador, sim.handle_world)
    error = np.linalg.norm(np.array(eff_pose[0:3]) - np.array(dum_pos[0:3]))

    if error < 0.01:
        sim.stopSimulation()
        print('Robô alcançou o objetivo, simulação encerrada!\n')
        break

    q = jointPos()
    joints_list.append(q)
    #jacob
    target = dum_pos - eff_pose
    jacobian = Jacob(q[0], q[1], q[2], q[3], q[4])
    man.append(np.sqrt(round(np.linalg.det(jacobian @ jacobian.T), 2))) #manipulabilidade
    j_inv_tgt = np.linalg.pinv(jacobian) @ target
    f = j_inv_tgt
    q = q + j_inv_tgt.flatten() * dt
    #atualizando posição
    #print('Posições: ', q, '\n')
    setJointPos(q)
    
    j = jointsTransforms(q[0], q[1], q[2], q[3], q[4])
    eff_pose = pos2pose(fKine(j))
    effpose_list.append(eff_pose)
    
    client.step()
    time.sleep(dt)

#PLOTS-----------------------------
t = np.arange(0, np.array(joints_list).shape[0]*dt, dt)
#Juntas
joints_list = np.array(joints_list)
plt.figure(facecolor= 'silver')
plt.title('Ângulos das Juntas')
for i in range(5):
    plt.plot(t, joints_list[:, i], label=f"Junta {i+1}")
plt.legend()
plt.show()

#manipulabilidade
plt.figure(facecolor='silver')
plt.title('Manipulabilidade')
plt.plot(t, np.array(man), label='manipulabilidade')
plt.legend()
plt.show()

#pose
lbls = ['x', 'y', 'z', 'wx', 'wy', 'wz']
effpose_list = np.array(effpose_list)
plt.figure(facecolor= 'silver')
plt.title('Pose do efetuador')
for i in range(6):
    plt.plot(t, effpose_list[:, i], label=lbls[i])
plt.legend()
plt.show()