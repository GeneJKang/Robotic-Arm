import numpy as np

axes = ['z', 'y', 'y', 'z', 'y', 'z'] #Using the ZYZ Euler angles for kinematic decoupling for the IK (later this is FK)
length = 0.2
toolOffset = 0.2


def get_joint_matrix(theta, L, axis):
    axis = axis.upper()
    #Creates a 4x4 homogenous transformation matrix for a single joint
    #Rotates around axis chosen

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    R = np.identity(3)  #Set up identity matrix 3x3

    #Populate 3x3 rotation matrix based on chosen axis
    if axis == 'X':
        R = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cos_t, -sin_t],
            [0.0, sin_t, cos_t]
        ])
    elif axis == 'Y':
        R = np.array([
            [cos_t, 0.0, sin_t],
            [0.0, 1.0, 0.0],
            [-sin_t, 0.0, cos_t]
        ])
    elif axis == 'Z':
        R = np.array([
            [cos_t, -sin_t, 0.0],
            [sin_t, cos_t, 0.0],
            [0.0, 0.0, 1.0]
        ])
    else:
        raise ValueError("Axis must be X, Y, or Z")
    

    #Define the position vector
    p = np.array([0, 0, L])

    #Assemble final 4x4 homogenous transformation matrix
    T = np.identity(4)
    T[0:3, 0:3] = R  #put rotation matrix on top left
    T[0:3, 3] = p #put translation vector on top right

    return T


def fk(thetaJ1, thetaJ2, thetaJ3, thetaJ4, thetaJ5, thetaJ6):

    T1 = get_joint_matrix(thetaJ1, length, axes[0])
    T2 = get_joint_matrix(thetaJ2, length, axes[1])
    T3 = get_joint_matrix(thetaJ3, length, axes[2])
    T4 = get_joint_matrix(thetaJ4, length, axes[3])
    T5 = get_joint_matrix(thetaJ5, length, axes[4])
    T6 = get_joint_matrix(thetaJ6, length, axes[5])
    Ttool = get_joint_matrix(0, length, axes[5])
    return T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ Ttool
