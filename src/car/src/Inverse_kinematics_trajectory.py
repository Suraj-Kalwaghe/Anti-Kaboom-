from sympy import symbols, cos, sin, pi, Matrix, pprint, simplify
import numpy as np
import matplotlib.pyplot as plt

# D-H parameters
# a_i = [0, 0.3, 0, 0, 0.015]
# d_i = [0.06, 0, 0, 0.325, 0]
# alpha_i = [pi/2, pi, pi/2, -pi/2, -pi/2]
theta = [0.3, 0.1, -0.1, -0.1, 0.09]

a_i = [0, 0.4, 0.4, 0.4, 0.05] 
d_i = [pi/2, 0, 0, 0, 0] 
alpha_i = [0.03, 0, 0, 0, 0]
# theta = [0.001,1.5708,0.0,0.0,0.0]

theta1, theta2, theta3, theta4, theta5 = symbols('theta1 theta2 theta3 theta4 theta5')

# Transformation matrices
def dhpara_transform_matrix(theta, d_i, a_i, alpha_i):
    return Matrix([
        [cos(theta), -sin(theta)*cos(alpha_i), sin(theta)*sin(alpha_i), a_i*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha_i), -cos(theta)*sin(alpha_i), a_i*sin(theta)],
        [0, sin(alpha_i), cos(alpha_i), d_i],
        [0, 0, 0, 1]
    ])

# D-H parameters transformation matrices for each link
T10 = dhpara_transform_matrix(theta1, d_i[0], a_i[0], alpha_i[0])
T21 = dhpara_transform_matrix(theta2, d_i[1], a_i[1], alpha_i[1])
T32 = dhpara_transform_matrix(theta3, d_i[2], a_i[2], alpha_i[2])
T43 = dhpara_transform_matrix(theta4, d_i[3], a_i[3], alpha_i[3])
T54 = dhpara_transform_matrix(theta5, d_i[4], a_i[4], alpha_i[4])

T20 = simplify(T10 * T21)
T30 = simplify(T20 * T32)
T40 = simplify(T30 * T43)
T50 = simplify(T40 * T54)

Tn0 = simplify(T50)

# printing the transformation matrices in their mathematical form(with correct symbols and notations)
# print("\n Final Transformation Matrix(Tn0)(Base frame to End-Effector frame)=\n")
# pprint(Tn0)

# Finding Jacobian using Method-2(partial differentiation)
P = Tn0[:3, 3]
z0 = T10[:3, 2]
z1 = T20[:3, 2]
z2 = T30[:3, 2]
z3 = T40[:3, 2]
z4 = Tn0[:3, 2]

Jacobian_matrix = Matrix([
    [P.diff(theta1), P.diff(theta2), P.diff(theta3), P.diff(theta4), P.diff(theta5)],
    [z0, z1, z2, z3, z4]
])

# print("Jacobian Matrix :\n ")
# pprint(Jacobian_matrix)

num_points = 190
omega = 2*pi/11
time_step = 0.001
damping_factor = 0.01
f_x = []
f_y = []
f_z = []

# Plotting the graph as a circular trajectory

theta = Matrix(theta)
for t in np.linspace(float(0.0), float(2*pi), num=num_points):
    X = 0.1*omega*cos(t)
    Z = -0.1*omega*sin(t)
    Vel = Matrix([X, 0, Z, 0.0, 0.0, 0.0])

    Jac_num = Jacobian_matrix.subs({theta1: theta[0], theta2: theta[1],
                                    theta3: theta[2], theta4: theta[3],
                                    theta5: theta[4]})
    Jac_num = simplify(Jac_num)
    pprint(Jac_num)
    # Damped least squares
    Jac_damped = Jac_num.T * (Jac_num * Jac_num.T + damping_factor**2 * Matrix.eye(6)).inv()
    theta_dot = (Jac_damped * Vel).evalf()
    theta = theta + theta_dot*time_step
    T50_vals = T50.evalf(subs={theta1: theta[0], theta2: theta[1],
                               theta3: theta[2], theta4: theta[3],
                               theta5: theta[4]})
    f_x.append(T50_vals[3])
    f_y.append(T50_vals[7])
    f_z.append(T50_vals[11])
plt.plot(f_z, f_x)
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.show()