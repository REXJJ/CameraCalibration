#!/usr/bin/env python

import numpy as np
import math

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([z, y, x])


def Generate_Transformation_Matrix(A, B):
    print("Inside function")
    assert len(A) == len(B)
    N = A.shape[0]; # number of rows
    #centroid calculation of points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))
    # Generation of covariance_matrix (3 x 3)
    H = np.dot(np.transpose(AA), BB)
    #singular value decompositon (SVD)
    # U = rotation matrix (3 x 3)
    # S = scaling_matrix (3 x 3)
    # Vt = rotation matrix (3 x 3)
    U, S, Vt = np.linalg.svd(H)
    # Matrix multiplication of rotation matrices (U, V) to generate combined --Rotation matrix (R)
    R = np.dot(Vt.T, U.T)
    # special reflection case
    if np.linalg.det(R) < 0:
       print ("Reflection detected")
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    # Translation matrix (position vector)
    t = -R.dot(centroid_A.T) + centroid_B.T
    #Generation of Transformation matrix
    transformation = np.concatenate((R, np.reshape(t, (3,1))),axis = 1)
    transformation = (np.concatenate((transformation.T, np.reshape([0, 0, 0, 1], (4, 1))), axis =1)).T
    
    np.set_printoptions(suppress=True)
    return R, t, transformation

def main(B,A):
    A=np.array(A)
    B=np.array(B)
    print("Transform Points")
    R,t,transformation=Generate_Transformation_Matrix(A,B)
    R=transformation[0:3,0:3]
    eul=rotationMatrixToEulerAngles(R)
    translation=transformation[0:3,3:4].transpose()[0]
    print("Translation")
    print(translation)
    static_tf=np.append(translation,eul)
    print("Static Transform")
    print(static_tf)
    f = open("../scripts/output.tmp", "w")
    temp=str(static_tf[0])
    for i in range(1,len(static_tf)):
        temp=temp+","+str(static_tf[i])
    f.write(temp)
    f.close()

if __name__ == "__main__":
    filepath = '../scripts/input.tmp'
    A=[]
    B=[]
    with open(filepath) as fp:
        line = fp.readline()
        count = int(line)
        for i in range(count):
            line = fp.readline()
            temp = line.split(",")
            temp= [float(x) for x in temp]
            A.append(temp)
        for i in range(count):
            line = fp.readline()
            temp = line.split(",")
            temp= [float(x) for x in temp]
            B.append(temp)
    main(A,B)

