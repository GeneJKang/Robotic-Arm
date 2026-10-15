import numpy as np
from arm_kinematics.ForwardKinematics import fk
from arm_kinematics.InverseKinematics import ik_solve
from arm_kinematics.InverseKinematics import wrap_angle

# Make a target we know is reachable, from known angles
target = fk(0.5, 0.8, 0.9, 0.3, 0.6, -0.4)

# Try all four arm configurations on the same target
for elbow_up in (True, False):
    for shoulder_flip in (False, True):
        sol = ik_solve(target, elbow_up, shoulder_flip)

        if sol is None:
            print(elbow_up, shoulder_flip, "-> None (rejected)")
        else:
            matches = np.allclose(fk(*sol), target)
            print(elbow_up, shoulder_flip, np.round(sol, 3), "matches target:", matches)

sol = ik_solve(target, True, False)
J1, J2, J3, J4, J5, J6 = sol
flipped = (J1, J2, J3, wrap_angle(J4 + np.pi), -J5, wrap_angle(J6 + np.pi))
print(np.allclose(fk(*sol), fk(*flipped)))
