import numpy as np


a = [0, 0.4, 0.4, 0.4, 0.05]  # Link lengths
alpha = [np.pi/2, 0, 0, 0, 0]  # Twist angles
d = [0.03, 0, 0, 0, 0]  # Offsets


def dh_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles):
    # Define your DH parameters
    # [theta, d, a, alpha]
    dh_params = [

        [joint_angles[0], 0.03, 0, np.pi/2],
        [joint_angles[1], 0, 0.4, 0],
        [joint_angles[2], 0, 0.4, 0],
        [joint_angles[3], 0, 0.4, 0],
        [joint_angles[4], 0, 0.05, 0],
        # [joint_angles[0], d[0], a[0], alpha[0]],
        # [joint_angles[1], d[1], a[1], alpha[1]],
        # [joint_angles[2], d[2], a[2], alpha[2]],
        # [joint_angles[3], d[3], a[3], alpha[3]],
        # [joint_angles[4], d[4], a[4], alpha[4]],

        # [joint_angles[0], 0.128, 0, np.pi / 2],
        # [joint_angles[1], 0, 0.6127, 0],
        # [joint_angles[2], 0, 0.5716, 0],
        # [joint_angles[3], 0.1639, 0, -np.pi / 2],
        # [joint_angles[4], 0.1157, 0, np.pi / 2],
        # [joint_angles[5], 0.0922, 0, 0]
    ]

    # Compute the transformation matrix for each joint
    T = np.eye(4)
    for params in dh_params:
        T = np.dot(T, dh_matrix(*params))

    # Extract the end-effector position
    end_effector_pos = T[:3, 3]
    # print(end_effector_pos)
    return end_effector_pos


def inverse_kinematics(target_pos, initial_joint_angles, max_iterations=100, tolerance=1e-8):
    joint_angles = initial_joint_angles.copy()

    for iteration in range(max_iterations):
        current_pos = forward_kinematics(joint_angles)
        error = target_pos - current_pos
        # print(error)

        if np.linalg.norm(error) < tolerance:
            print(f"Inverse Kinematics Converged in {iteration} iterations.")
            return joint_angles

        # Compute the Jacobian matrix
        delta = 1e-6
        J = np.zeros((3, 5))

        for i in range(5):
            joint_angles_delta = joint_angles.copy()
            joint_angles_delta[i] += delta
            pos_delta = forward_kinematics(joint_angles_delta)
            J[:, i] = (pos_delta - current_pos) / delta
        

        # Update joint angles using the pseudo-inverse
        J_inv = np.linalg.pinv(J, rcond=1e-8)
        
        joint_angles = joint_angles + np.dot(J_inv, error)
        # print(J_inv)
    print("Inverse Kinematics did not converge.")
    return None


target_position = np.array([0.000, 0.000, 0.713])  #  end-effector position
initial_joint_angles = np.array([0,1.57,0,0,0])
# initial_joint_angles = np.array([1.5708,1.5708, -0.785, -0.785, 0])  # Initial joint angles

result_angles = inverse_kinematics(target_position, initial_joint_angles)

my_list = result_angles
my_formatted_list = [ '%.3f' % elem for elem in my_list ]
# Print the result
if result_angles is not None:
    print("Resulting Joint Angles:", my_formatted_list)
    # print("Resulting Joint Angles:", result_angles)
else:
    print("Inverse Kinematics did not converge.")