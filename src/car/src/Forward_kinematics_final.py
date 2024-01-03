import sympy as sp
from sympy import symbols, cos, sin, pi, simplify, Matrix, pprint, diff, shape
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math


theta, theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta theta1 theta2 theta3 theta4 theta5 theta6')
t, omega = symbols('t omega')

# theta_arm = [0, -1.5708, 1.5708, -1.5708, 1.5708, -0.0174533]

#DH parameters
# a = [0, 0.6127, 0.5716, 0, 0, 0]
# d = [0.128, 0, 0, 0.1639, 0.1157, 0.0922]
# alpha = [sp.pi/2, 0, 0, -sp.pi/2, sp.pi/2, 0]

a = [0, 0.4, 0.4, 0.4, 0.05]  # Link lengths
alpha = [sp.pi/2, 0, 0, 0, 0]  # Twist angles
d = [0.03, 0, 0, 0, 0]  # Offsets



# DH transformation matrix (standard)
def DH_matrix(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(theta, d, a, alpha): 
    T0_1 = DH_matrix(theta[0], d[0], a[0], alpha[0])
    T1_2 = DH_matrix(theta[1], d[1], a[1], alpha[1])
    T2_3 = DH_matrix(theta[2], d[2], a[2], alpha[2])
    T3_4 = DH_matrix(theta[3], d[3], a[3], alpha[3])
    T4_5 = DH_matrix(theta[4], d[4], a[4], alpha[4])
    # T5_6 = DH_matrix(theta[5], d[5], a[5], alpha[5])

    T0_2 = simplify(T0_1 * T1_2) 
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_n = simplify(T0_5)
    # T0_n = simplify(T0_6)

    position = T0_n[:3, 3]

    return np.array(position.evalf(subs={theta1: theta[0], theta2: theta[1],
                                          theta3: theta[2], theta4: theta[3],
                                          theta5: theta[4]}), dtype=float)



# initial_joint_angles = np.array([0,0,0,0,0]) 
# initial_joint_angles = np.array([0.000, 1.5708, 0.0 , 0.00, 0.00]) # Joint angles in radians
# initial_joint_angles = np.array([-0.6109, 1.0472, -0.785 ,-1.0472,  -0.5236])
initial_joint_angles = np.array([1.5708,1.5708, -0.785, -0.785, 0])
# initial_joint_angles = np.array([0,0.785, 1.5708, -0.785, 0])

position_result = forward_kinematics(initial_joint_angles, d, a, alpha)
my_list = position_result
my_formatted_list = ['%.3f' % elem for elem in my_list]

print("Forward Kinematics Result:")
print("Position:", my_formatted_list)

# # inverse Kinematics
# T0_1 = DH_matrix(theta[0], d[0], a[0], alpha[0])
# T1_2 = DH_matrix(theta[1], d[1], a[1], alpha[1])
# T2_3 = DH_matrix(theta[2], d[2], a[2], alpha[2])
# T3_4 = DH_matrix(theta[3], d[3], a[3], alpha[3])
# T4_5 = DH_matrix(theta[4], d[4], a[4], alpha[4])
# T5_6 = DH_matrix(theta[5], d[5], a[5], alpha[5])

# T0_2 = simplify(T0_1 * T1_2) 
# T0_3 = simplify(T0_2 * T2_3)
# T0_4 = simplify(T0_3 * T3_4)
# T0_5 = simplify(T0_4 * T4_5)
# T0_6 = simplify(T0_5 * T5_6)
# T0_n = simplify(T0_6)

# position = T0_n[:3, 3]

# def inverse_kinematics(target_position, initial_joint_angles, max_iterations=100, tolerance=1e-6):
#     q = initial_joint_angles

#     for iteration in range(max_iterations):
#         # Forward kinematics to get the current end-effector position
#         current_position = T0_n.evalf(subs={theta1: q[0], theta2: q[1],
#                                             theta3: q[2], theta4: q[3],
#                                             theta5: q[4], theta6: q[5]})[:3, 3]

#         # Error in end-effector position
#         error = target_position - current_position

#         # Check if the error is below the tolerance
#         if np.linalg.norm(error) < tolerance:
#             print(f"Inverse Kinematics Converged in {iteration} iterations.")
#             return q

#         # Calculate joint velocities using the Jacobian pseudo-inverse
#         J_n = J.subs({theta1: q[0], theta2: q[1], theta3: q[2], theta4: q[3], theta5: q[4], theta6: q[5]})
#         J_n = simplify(J_n)

#         J_inv_n = J_n.pinv()
#         qdot = ((J_inv_n * error) % (2 * np.pi)).evalf()

#         # Update joint angles
#         q = (q + qdot).evalf()

#     print("Inverse Kinematics did not converge.")
#     return None

# # Example Usage
# target_position = np.array([0.5, 0.5, 0.5])  # Replace with your desired end-effector position
# initial_joint_angles = np.array(q_arm)  # Initial joint angles

# result_angles = inverse_kinematics(target_position, initial_joint_angles)

# # Print the result
# if result_angles is not None:
#     print("Resulting Joint Angles:", result_angles)
# else:
#     print("Inverse Kinematics did not converge.")








#DH transformation Matrix for each of the link
# T0_1 = DH_matrix(theta1, d[0], a[0], alpha[0])
# T1_2 = DH_matrix(theta2, d[1], a[1], alpha[1])
# T2_3 = DH_matrix(theta3, d[2], a[2], alpha[2])
# T3_4 = DH_matrix(theta4, d[3], a[3], alpha[3])
# T4_5 = DH_matrix(theta5, d[4], a[4], alpha[4])
# T5_6 = DH_matrix(theta6, d[5], a[5], alpha[5])

# T0_2 = simplify(T0_1 * T1_2) 
# T0_3 = simplify(T0_2 * T2_3)
# T0_4 = simplify(T0_3 * T3_4)
# T0_5 = simplify(T0_4 * T4_5)
# T0_6 = simplify(T0_5 * T5_6)
# T0_n = simplify(T0_6)


# P = T0_n[:3, 3]
# z0 = T0_1[:3, 2]
# z1 = T0_2[:3, 2]
# z2 = T0_3[:3, 2]
# z3 = T0_4[:3, 2]
# z4 = T0_5[:3, 2]
# z5 = T0_n[:3, 2]

# J = Matrix([
#     [P.diff(theta1), P.diff(theta2), P.diff(theta3), P.diff(theta4), P.diff(theta5), P.diff(theta6)],
#     [z0, z1, z2, z3, z4, z5]
# ])
