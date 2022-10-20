import numpy as np
import math as m
import roboticstoolbox as rtb

l1 = 0.475
l2 = 0.4

# direct kinematics

def fkine(theta_1, theta_2, theta_3, d_4):
  return np.array([[m.cos(theta_3+theta_2+theta_1), m.sin(theta_3+theta_2+theta_1), 0, l1*m.cos(theta_1)+l2*m.cos(theta_2+theta_1)],
                   [m.sin(theta_3+theta_2+theta_1), -m.cos(theta_3+theta_2+theta_1), 0, l1*m.sin(theta_1)+l2*m.sin(theta_2+theta_1)],
                   [0, 0, -1, -d_4],
                   [0, 0, 0, 1]])


t01 = rtb.robot.DHLink(a = l1, offset=0)
t12 = rtb.robot.DHLink(a = l2, offset=0)
t23 = rtb.robot.DHLink(alpha = m.pi, offset=0)
t34 = rtb.robot.DHLink(sigma = 1, qlim=[0, 0.1])
scara = rtb.robot.DHRobot([t01, t12, t23, t34], name = 'SCARA')
print(scara)

print("LETRA A")
T1 = fkine(0, 0, 0, 0)
a = [0, 0, 0, 0]
print(T1)
fkine_scara1 = rtb.DHRobot.fkine(scara,a)
print(fkine_scara1)
print("\n\n")

print("LETRA B")
T2 = fkine(m.pi/2, m.pi/2, 0, 0)
b = [m.pi/2, m.pi/2, 0, 0]
print(T2)
fkine_scara2 = rtb.DHRobot.fkine(scara,b)
print(fkine_scara2)
print("\n\n")


print("LETRA C")
T3 = fkine(m.pi/2, m.pi/2 * -1, 0, 0.05)
c = [m.pi/2, m.pi/2 * -1, 0, 0.05]
print(T3)
fkine_scara3 = rtb.DHRobot.fkine(scara,c)
print(fkine_scara3)
print("\n\n")
