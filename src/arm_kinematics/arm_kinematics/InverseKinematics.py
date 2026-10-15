import numpy as np


pi = np.pi
L = 0.2

#These limits are +- and in radians
J_limits = [pi, pi/2, 2.6, pi, 2.1, pi]


def within_limits(angles):
    for angle, limit in zip(angles, J_limits):
        if (-limit > angle or limit < angle):
            return False
    return True

def wrap_angle(angle):
    return (angle + pi) % (2 * pi) - pi




def ik_solve(target_pose, elbow_up, shoulder_flip):
    #Extract global orientation R and position P
    R_target = target_pose[0:3, 0:3]
    P_target = target_pose[0:3, 3]


    #Compute wrist center
    #Wrist is T6 + Ttool so its L + L = 2 * L
    #Wrist center is shifted back along the Z-axis of the end effector by d6
    d6 = 2 * L
    z_ee = R_target[:, 2]   #end effector Z-axis
    Wc = P_target - d6 * z_ee
    xc, yc, zc = Wc[0], Wc[1], Wc[2]


    #Joint 1, 2, 3 IK position
    #Joint 1 Base Z
    J1 = np.arctan2(yc, xc)

    #Project wrist center to the planar 2R arm coordinates aka 3D to 2D problem
    #Project wrist center as planar 2R arm coordinate frame with X_arm, Y_arm
    r = np.sqrt(xc**2 + yc**2)
    x_arm = r
    y_arm = zc - 2 * L  #Subtracting initial translation of T1

    if (shoulder_flip):
        x_arm = -x_arm
        J1 = wrap_angle(J1 + pi) #This adds pi to J1 and its wraps around instead of going out of bounds

    #J2 and J3 move along Z by L so Link 2 length is L and Link3 length is L
    L2 = L
    L3 = 2 * L

    #Law of cosines to solve J3
    D_sq = x_arm**2 + y_arm**2
    cos_J3 = (D_sq - L2**2 - L3**2) / (2 * L2 * L3)
    cos_J3_clipped = np.clip(cos_J3, -1.0, 1.0)

    if (not np.isclose(cos_J3_clipped, cos_J3)):    #Out of bounds when position is too far or too close
        return None
    

    #Elbow-up configuration
    J3 = np.arccos(cos_J3_clipped)
    if (not elbow_up):
        J3 = -J3

    #Joint 2
    alpha = np.arctan2(y_arm, x_arm)
    beta = np.arctan2(-L3 * np.sin(J3), L2 + L3 * np.cos(J3))
    J2 = np.pi/2 - (alpha - beta)



    #Joints 4, 5, 6 IK position
    #Extracting the 3x3 rotations from fk
    def rot_z(t): return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])
    def rot_y(t): return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])

    R0_1 = rot_z(J1)
    R1_2 = rot_y(J2)
    R2_3 = rot_y(J3)
    R0_3 = R0_1 @ R1_2 @ R2_3

    #Isolate remaining rotation for ZYZ wrist (J4, J5, J6)
    #Transpose R0_3
    R3_6 = R0_3.T @ R_target

    #Extract ZYZ euler angles from R3_6 rotation matrix
    r1_3 = R3_6[0,2]
    r2_3 = R3_6[1, 2]
    r3_3 = R3_6[2, 2]
    r3_1 = R3_6[2, 0]
    r3_2 = R3_6[2, 1]

    #J5 is rotation around inner Y axis of the wrist
    J5 = np.arccos(np.clip(r3_3, -1.0, 1.0))

    if np.isclose(J5, 0.0):
        #Gimbal lock handling
        J4 = 0.0
        J6 = np.arctan2(-R3_6[0, 1], R3_6[0, 0])
    else:
        J4 = np.arctan2(r2_3, r1_3)
        J6 = np.arctan2(r3_2, -r3_1)

    if (within_limits([J1, J2, J3, J4, J5, J6])):
        return J1, J2, J3, J4, J5, J6
    else:
        flipped = (J1, J2, J3, wrap_angle(J4 + pi), -J5, wrap_angle(J6 + pi))
        if within_limits(flipped):
            return flipped
        else:
            return None


def ik(target_pose):

    configs = [
        (True, False),
        (False, False),
        (True, True),
        (False, True),
    ]

    for elbow_up, shoulder_flip in configs:
        result = ik_solve(target_pose, elbow_up,  shoulder_flip)
        if (result is not None):
            break
    return result

