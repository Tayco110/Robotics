import numpy as np
import math as m
from zmqRemoteApi import RemoteAPIClient
import roboticstoolbox as rtb
import time
import sim

l1 = 0.475
l2 = 0.4

# direct kinematics

def fkine(theta_1, theta_2, theta_3, d_4):
  return np.array([[m.cos(theta_3+theta_2+theta_1), m.sin(theta_3+theta_2+theta_1), 0, l1*m.cos(theta_1)+l2*m.cos(theta_2+theta_1)],
                   [m.sin(theta_3+theta_2+theta_1), -m.cos(theta_3+theta_2+theta_1), 0, l1*m.sin(theta_1)+l2*m.sin(theta_2+theta_1)],
                   [0, 0, -1, -d_4],
                   [0, 0, 0, 1]])

def matrix_transformation(thetha, d, a, alpha):
    return np.array(
        [
            [np.cos(thetha), -np.sin(thetha) * np.cos(alpha), np.sin(thetha) * np.sin(alpha), a * np.cos(thetha)],
            [np.sin(thetha), np.cos(thetha) * np.cos(alpha), -np.cos(thetha) * np.sin(alpha), a * np.sin(thetha)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )

def invFkine(Matrix, l1, l2, theta1_ref, theta2_ref, theta3_ref):
    # x, y, z = Matrix[0:3, -1]
    print(Matrix)
    x, y = Matrix[0:2, -1]
    z = 0
    phi = np.arctan2(Matrix[1, 0], Matrix[0, 0])
    if z != 0:
        raise Exception()
    c2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if c2 > 1 or c2 < -1:
        raise Exception()
    s2_1 = np.sqrt(1 - (c2 ** 2))
    s2_2 = -np.sqrt(1 - (c2 ** 2))
    theta2_1 = np.arctan2(s2_1, c2)
    theta2_2 = np.arctan2(s2_2, c2)

    k1 = l2 * c2 + l1
    k2_1 = l2 * s2_1
    k2_2 = l2 * s2_2
    theta1_1 = np.arctan2(y, x) - np.arctan2(k2_1, k1)
    theta1_2 = np.arctan2(y, x) - np.arctan2(k2_2, k1)

    theta3_1 = phi - theta1_1 - theta2_1
    theta3_2 = phi - theta1_2 - theta2_2

    d4 = -Matrix[2, -1]

    opt1 = (theta1_ref - theta1_1)**2 + (theta2_ref - theta2_1)**2 + (theta3_ref - theta3_1)**2
    opt2 = (theta1_ref - theta1_2)**2 + (theta2_ref - theta2_2)**2 + (theta3_ref - theta3_2)**2

    if opt1 <= opt2:
        return theta1_1, theta2_1, theta3_1, d4
    return theta1_2, theta2_2, theta3_2, d4

#Montando robo
t01 = rtb.robot.DHLink(a = l1, offset=0)
t12 = rtb.robot.DHLink(a = l2, offset=0)
t23 = rtb.robot.DHLink(alpha = m.pi, offset=0)
t34 = rtb.robot.DHLink(sigma = 1, qlim=[0, 0.1])
scara = rtb.robot.DHRobot([t01, t12, t23, t34], name = 'SCARA')

sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 4200, True, True, 5000, 5)  # Connect to CoppeliaSim

client = RemoteAPIClient()
sim_ = client.getObject('sim')

if clientID != -1:
    print('Connected to remote API server\n')
    time.sleep(0.02)
    err, dummy = sim.simxGetObjectHandle(clientID, 'reference', sim.simx_opmode_oneshot_wait)
    err, floor = sim.simxGetObjectHandle(clientID, 'Floor', sim.simx_opmode_oneshot_wait)
    err, scara_coppeliasim = sim.simxGetObjectHandle(clientID, 'MTB', sim.simx_opmode_oneshot_wait)
    err, eixo1 = sim.simxGetObjectHandle(clientID, 'eixo1',sim.simx_opmode_oneshot_wait)
    err, eixo2 = sim.simxGetObjectHandle(clientID, 'eixo2',sim.simx_opmode_oneshot_wait)
    err, eixo3 = sim.simxGetObjectHandle(clientID, 'eixo3',sim.simx_opmode_oneshot_wait)
    err, eixo4 = sim.simxGetObjectHandle(clientID, 'eixo4',sim.simx_opmode_oneshot_wait)

    # Criando stream de dados
    err, position_dummy = sim.simxGetObjectPosition(clientID, dummy, floor, sim.simx_opmode_streaming)
    err, orientation_dummy = sim.simxGetObjectOrientation(clientID, dummy, floor, sim.simx_opmode_streaming)

    time.sleep(5)

    err, position_dummy = sim.simxGetObjectPosition(clientID, dummy, floor, sim.simx_opmode_buffer)
    err, orientation_dummy = sim.simxGetObjectOrientation(clientID, dummy, floor, sim.simx_opmode_buffer)

    orientation_dummy_degree = [d * (180 / m.pi) for d in orientation_dummy]
    print(f"Position Dummy: {position_dummy}\n")
    print(f"Orientation Dummy: {orientation_dummy_degree}\n")

    matrix_transformation = sim_.getObjectMatrix(dummy, floor)
    matrix_transformation= np.array(matrix_transformation).reshape((3, 4))

    move_position = invFkine(matrix_transformation, l1, l2, 0, 0, 0)

    sim.simxSetJointPosition(clientID,eixo1, move_position[0], sim.simx_opmode_oneshot_wait)
    sim.simxSetJointPosition(clientID,eixo2, move_position[1], sim.simx_opmode_oneshot_wait)
    sim.simxSetJointPosition(clientID,eixo3, move_position[2], sim.simx_opmode_oneshot_wait)
    sim.simxSetJointPosition(clientID,eixo4, move_position[3], sim.simx_opmode_oneshot_wait)

    sim.simxGetPingTime(clientID)

    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')

