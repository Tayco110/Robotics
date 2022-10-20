from zmqRemoteApi import RemoteAPIClient
import math
import numpy as np

client = RemoteAPIClient()
sim = client.getObject('sim')

def rad_to_degree(matrix):
    orientation_dummy_degree = []
    for i in matrix:
        orientation_dummy_degree.append(i * (180 / math.pi))  
    return orientation_dummy_degree

def lerpose():

    dummy = sim.getObject('/Dummy')
    print("\nPosition: ", sim.getObjectPosition(dummy, -1))

    orientation_dummy =  sim.getObjectOrientation(dummy, -1)
    orientation_dummy_degree = rad_to_degree(orientation_dummy)
    print("\nOrientation: ", orientation_dummy_degree)

    matrix = sim.getObjectMatrix(dummy, -1)
    matrix_formatada = np.array(matrix).reshape((3,4))
    print("\nHomogeneus Transformation Matrix: \n")
    print(matrix_formatada)

while True:
    print('\n1 - Read Dummy')
    op = input()
    if op == '1': lerpose()
    else: break