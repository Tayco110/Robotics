import time
from xml.dom import xmlbuilder
import numpy as np
from sympy import true
import matplotlib.pyplot as plt

from zmqRemoteApi import RemoteAPIClient

TOLERANCE = 0.01
Ts = 0.0500

np.set_printoptions(suppress=True, precision=4)

def c(x):return np.cos(x) 
def s(x):return np.sin(x) 

def t0_1(tetha):
    return np.array([
        c(tetha), 0, -s(tetha), 0,
        s(tetha), 0,  c(tetha), 0,
        0,       -1,        0,  0.11,
        0,        0,        0,  1
    ]).reshape(4, 4)

def t1_2(tetha):
    return np.array([
        c((2*tetha-np.pi)/2), -s((2*tetha-np.pi)/2), 0, 0.125*c((2*tetha-np.pi)/2),
        s((2*tetha-np.pi)/2),  c((2*tetha-np.pi)/2), 0, 0.125*s((2*tetha-np.pi)/2),
                           0,                     0, 1,                          0,
                           0,                     0, 0,                          1
    ]).reshape(4, 4)

def t2_3(tetha):
    return np.array([
        c((2*tetha+np.pi)/2), -s((2*tetha+np.pi)/2), 0, 0.096*c((2*tetha+np.pi)/2),
        s((2*tetha+np.pi)/2),  c((2*tetha+np.pi)/2), 0, 0.096*s((2*tetha+np.pi)/2),
                           0,                     0, 1,                          0,
                           0,                     0, 0,                          1
    ]).reshape(4, 4)

def t3_4(tetha):
    return np.array([
        c((2*tetha+np.pi)/2), 0,   s((2*tetha+np.pi)/2), -0.0275*c((2*tetha+np.pi)/2),
        s((2*tetha+np.pi)/2), 0,  -c((2*tetha+np.pi)/2), -0.0275*s((2*tetha+np.pi)/2),
                           0, 1,                      0,                          0,
                           0, 0,                      0,                          1
    ]).reshape(4, 4)

def t4_5(tetha):
    return np.array([
        c(tetha), -s(tetha), 0, 0,
        s(tetha),  c(tetha), 0, 0,
               0,         0, 1, 0.065,
               0,         0, 0, 1
    ]).reshape(4, 4)

def fkine(q):
    return t0_1(q[0]) @ t1_2(q[1]) @ t2_3(q[2]) @ t3_4(q[3]) @ t4_5(q[4])

def quat2euler(h):
    roll = np.arctan2(2*(h[0]*h[1] + h[2]*h[3]), 1 - 2*(h[1]**2 + h[2]**2))
    pitch = np.arcsin(2*(h[0]*h[2] - h[3]*h[1]))
    yaw = np.arctan2(2*(h[0]*h[3] + h[1]*h[2]), 1 - 2*(h[2]**2 + h[3]**2))

    return (roll, pitch, yaw)

def matrix_union(A, B):
    for a, b in zip(A, B):
        yield [*a, *b]

def jacobian(q):
    r0_1 = np.array([c(q[0]), 0, -s(q[0]),
                     s(q[0]), 0,  c(q[0]),
                             0,-1,        0]).reshape(3, 3)

    r1_2 = np.array([c((2*q[1]-np.pi)/2), -s((2*q[1]-np.pi)/2), 0,
                     s((2*q[1]-np.pi)/2),  c((2*q[1]-np.pi)/2), 0,
                                         0,                     0, 1]).reshape(3, 3)

    r2_3 = np.array([c((2*q[2]+np.pi)/2), -s((2*q[2]+np.pi)/2), 0,
                     s((2*q[2]+np.pi)/2),  c((2*q[2]+np.pi)/2), 0,
                                         0,                     0, 1]).reshape(3, 3)

    r3_4 = np.array([c((2*q[3]+np.pi)/2), 0,   s((2*q[3]+np.pi)/2),
                     s((2*q[3]+np.pi)/2), 0,  -c((2*q[3]+np.pi)/2),
                                        0,  1,                      0]).reshape(3, 3)

    r4_5 = np.array([c(q[4]), -s(q[4]), 0,
                     s(q[4]),  c(q[4]), 0,
                            0,         0, 1]).reshape(3, 3)

    ############################################################################################

    p0 = np.array([0, 0, 0]).reshape(3, 1)
    
    p1 = t0_1(q[0])[0:3, -1].reshape(3, 1)

    p2 = (t0_1(q[0]) @ t1_2(q[1]))[0:3, -1].reshape(3, 1)

    p3 = (t0_1(q[0]) @ t1_2(q[1]) @  t2_3(q[2]))[0:3, -1].reshape(3, 1)

    p4 = (t0_1(q[0]) @ t1_2(q[1]) @ t2_3(q[2]) @ t3_4(q[3]))[0:3, -1].reshape(3, 1)

    p5 = (t0_1(q[0]) @ t1_2(q[1]) @ t2_3(q[2]) @ t3_4(q[3]) @ t4_5(q[4]))[0:3, -1].reshape(3, 1)

    ############################################################################################

    z0 = np.array([0, 0, 1]).reshape(3, 1)

    z1 = np.dot(r0_1, z0)

    z2 = np.dot(r0_1, r1_2)
    z2 = np.dot(z2, z0)

    z3 = np.dot(r0_1, r1_2)
    z3 = np.dot(z3, r2_3)
    z3 = np.dot(z3, z0)

    z4 = np.dot(r0_1, r1_2)
    z4 = np.dot(z4, r2_3)
    z4 = np.dot(z4, r3_4)
    z4 = np.dot(z4, z0)

    z5 = np.dot(r0_1, r1_2)
    z5 = np.dot(z5, r2_3)
    z5 = np.dot(z5, r3_4)
    z5 = np.dot(z5, r4_5)
    z5 = np.dot(z5, z0)

    x0 = np.cross(z0.T, (p5-p0).T).T
    x1 = np.cross(z1.T, (p5-p1).T).T
    x2 = np.cross(z2.T, (p5-p2).T).T
    x3 = np.cross(z3.T, (p5-p3).T).T
    x4 = np.cross(z4.T, (p5-p4).T).T

    #concatena horizontalmente
    result_cima1 = list(matrix_union(x0, x1))
    result_cima2 = list(matrix_union(x2, x3))
    result_cima3 = list(matrix_union(result_cima2, x4))

    linha_de_cima = list(matrix_union(result_cima1, result_cima3))

    result_baixo1 = list(matrix_union(z0, z1))
    result_baixo2 = list(matrix_union(z2, z3))
    result_baixo3 = list(matrix_union(result_baixo2, z4))

    linha_de_baixo = list(matrix_union(result_baixo1, result_baixo3))

    #concatena verticalmente
    result = np.concatenate((linha_de_cima, linha_de_baixo))

    return result

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

dummy_handle = sim.getObject('/Dummy')

position = sim.getObjectPosition(dummy_handle, -1)
orientation = sim.getObjectQuaternion(dummy_handle, -1)

quat = np.array([orientation[3], orientation[0], orientation[1], orientation[2]]) # Remember: getObjectQuaternion has real part as last element

(alpha, beta, gamma) = sim.getObjectOrientation(dummy_handle, -1)
(yaw, pitch, roll) = sim.alphaBetaGammaToYawPitchRoll(alpha, beta, gamma)

# Montando vetor de pose desejada
X_d = np.array([[position[0]], [position[1]], [position[2]], [roll], [pitch], [yaw]])

junta1 = sim.getObject('/theta1')
junta2 = sim.getObject('/theta2')
junta3 = sim.getObject('/theta3')
junta4 = sim.getObject('/theta4')
junta5 = sim.getObject('/theta5')

joints = []
joints.append(junta1)
joints.append(junta2)
joints.append(junta3)
joints.append(junta4)
joints.append(junta5)

theta1 = sim.getJointPosition(joints[0])
theta2 = sim.getJointPosition(joints[1])
theta3 = sim.getJointPosition(joints[2])
theta4 = sim.getJointPosition(joints[3])
d4 = sim.getJointPosition(joints[4])

q = np.array([theta1, theta2, theta3, theta4, d4])

q_inicial = q

T = fkine(q)

R = T[0:3, 0:3]

for i in range(3):
    for j in range(3):
        if (np.abs(R[i][j]) < 0.01):
            R[i][j] = 0.0

alpha = np.arctan2(R[1][0], R[0][0])

beta = np.arctan2(-R[2][0], np.sqrt( (R[2][1]**2) + (R[2][2]**2)))

gamma = np.arctan2(R[2][1], R[2][2])

# Montando vetor de pose atual
X_m = np.vstack([T[0:3, -1].reshape(3,1), np.array([alpha, beta, gamma]).reshape((3, 1))])

print('goal:')

cond = 0

manipulabilidade = []

efetuador_x = []
efetuador_y = []
efetuador_z = []

theta_1 = []
theta_2 = []
theta_3 = []
theta_4 = []
theta_5 = []

efetuador = sim.getObject('/end_effector_visual')

Tempo = np.arange(0, 15, Ts)



while (cond < 15):
    error = X_d - X_m
    J = jacobian(q)
    

    #Escrevendo a manipulabilidade
    try:
        manipulabilidade.append(np.sqrt(np.linalg.det(J@J.T)))
    except:
        pass
    
    (x, y, z)  = sim.getObjectPosition(efetuador, -1)
    
    efetuador_x.append(x)
    efetuador_y.append(y)
    efetuador_z.append(z)

    theta_1.append(q[0])
    theta_2.append(q[1])
    theta_3.append(q[2])
    theta_4.append(q[3])
    theta_5.append(q[4])


    J_pinv = (np.transpose(J) @ np.linalg.inv(J @ np.transpose(J) + 0.5**2 * np.eye(6))) # Pseuinversa amortecida
    dq = J_pinv @ error # Cinemática diferencial
    dq = dq.reshape((5, ))

    q = q + dq*Ts

    sim.setJointTargetPosition(joints[0], q[0])
    sim.setJointTargetPosition(joints[1], q[1])
    sim.setJointTargetPosition(joints[2], q[2])
    sim.setJointTargetPosition(joints[3], q[3])
    sim.setJointTargetPosition(joints[4], q[4])

    time.sleep(Ts)

    theta1 = sim.getJointPosition(joints[0])
    theta2 = sim.getJointPosition(joints[1])
    theta3 = sim.getJointPosition(joints[2])
    theta4 = sim.getJointPosition(joints[3])
    d4 = sim.getJointPosition(joints[4])

    q = np.array([theta1, theta2, theta3, theta4, d4])

    T = fkine(q)

    R = T[0:3, 0:3]

    for i in range(0, 3):
        for j in range(0, 3):
            if (np.abs(R[i][j]) < 0.1):
                R[i][j] = 0.0

    alpha = np.arctan2(R[1][0], R[0][0])

    beta = np.arctan2(-R[2][0], np.sqrt( (R[2][1]**2) + (R[2][2]**2)))

    gamma = np.arctan2(R[2][1], R[2][2])

    X_m = np.vstack([T[0:3, -1].reshape(3,1), np.array([alpha, beta, gamma]).reshape((3, 1))])

    position = sim.getObjectPosition(dummy_handle, -1)
    orientation = sim.getObjectQuaternion(dummy_handle, -1)

    quat = np.array([orientation[3], orientation[0], orientation[1], orientation[2]]) # Remember: getObjectQuaternion has real part as last element

    (alpha, beta, gamma) = sim.getObjectOrientation(dummy_handle, -1)
    (yaw, pitch, roll) = sim.alphaBetaGammaToYawPitchRoll(alpha, beta, gamma)

    X_d = np.array([[position[0]], [position[1]], [position[2]], [roll], [pitch], [yaw]])

    cond = cond + Ts

print("MATRIZ JACOBIANA INICIAL:\n", jacobian(q_inicial))
print("\nMATRIZ JACOBIANA FINAL:\n", jacobian(q))

plt.scatter(Tempo, manipulabilidade, s = 1.5, c = 'k')
plt.title('Manipulabilidade do sistema')
plt.grid(0.5)
plt.xlabel('Tempo(s)')
plt.ylabel('Manipulabilidade')
plt.show()

plt.plot(Tempo, efetuador_x,'r', label = 'Posição X do efetuador', linewidth = 1.5)
plt.plot(Tempo, efetuador_y,'g', label = 'Posição Y do efetuador', linewidth = 1.5)
plt.plot(Tempo, efetuador_z,'b', label = 'Posição Z do efetuador', linewidth = 1.5)
plt.legend(loc='best', framealpha=1)
plt.title('Posição das coordenadas')
plt.grid(0.5)
plt.xlabel('Tempo(s)')
plt.ylabel('Coordenadas')
plt.show()

plt.plot(Tempo, theta_1,'r', label = 'Theta 1', linewidth = 1.5)
plt.plot(Tempo, theta_2,'g', label = 'Theta 2', linewidth = 1.5)
plt.plot(Tempo, theta_3,'b', label = 'Theta 3', linewidth = 1.5)
plt.plot(Tempo, theta_4,'y', label = 'Theta 4', linewidth = 1.5)
plt.plot(Tempo, theta_5,'k', label = 'Theta 5', linewidth = 1.5)
plt.legend(loc='best', framealpha=1)
plt.title('Posição das Juntas')
plt.grid(0.5)
plt.xlabel('Tempo(s)')
plt.ylabel('Juntas')
plt.show()