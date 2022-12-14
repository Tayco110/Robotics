import time
from xml.dom import xmlbuilder
import numpy as np
from sympy import true
import matplotlib.pyplot as plt

from zmqRemoteApi import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

joints = []
positions = []
forces = [0, -0.1895306259393692, -0.18952909111976624, 0, 0]

sim.startSimulation()

junta1 = sim.getObject('/theta1')
junta2 = sim.getObject('/theta2')
junta3 = sim.getObject('/theta3')
junta4 = sim.getObject('/theta4')
junta5 = sim.getObject('/theta5')

joints.append(junta1)
joints.append(junta2)
joints.append(junta3)
joints.append(junta4)
joints.append(junta5)

# theta1 = sim.getPosition(joints[0])
# theta2 = sim.getJointPosition(joints[1])
# theta3 = sim.getJointPosition(joints[2])
# theta4 = sim.getJointPosition(joints[3])
# d4 = sim.getJointPosition(joints[4])

#forces = [3.183837568587755e-11, 0.1895306259393692+9.8, 0.18952909111976624+9.8, 1.1968174362664286e-07, 1.3940850469528723e-15]
#forces = [0, -0.19, -0.19, 1.1968174362664286e-07, 0]
#forces = [0, 0, -0.19, 0, 0]

#k = 0.1

# for i in range (len(joints)):
#     positions = sim.getObjectPosition(joints[i], -1)
#     sim.addForce(joints[i], positions, forces)

while(true):
    #sim.setJointTargetForce(joints[1],forces[1])
    #sim.setJointTargetForce(joints[2],forces[2])
    for i in range (len(joints)):
        sim.setJointTargetForce(joints[i],forces[i])
        print(sim.getJointForce(joints[i]))
        time.sleep(0.05)
        #positions = sim.getObjectPosition(joints[i], -1)
        # print(joints[i])
        # print(len(positions))