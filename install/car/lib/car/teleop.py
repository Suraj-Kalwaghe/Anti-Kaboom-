#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard
import sympy as sp

# Define key codes
LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 0.15
MANU_ANGLE = 0.05
CLAW_ANGLE = 0.05
DISK_MOVEMENT = 0.05

MOVE_LINK1 = 0.25
MOVE_LINK2 = 0.25
MOVE_LINK3 = 0.25
MOVE_LINK4 = 0.25
arr = [ 6.66369199, 0.09487723, 26.88209519, -7.35755427, 9.3351371]
class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
        # self.arm_positions_pub = self.create_publisher(Float64MultiArray, '/manu_position_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Robot Control with keyboard
        
        Navigation Keys:

             W

        A    S    D

        Q : STOP

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        # arm_positions = Float64MultiArray()

        linear_vel=0.0
        steer_angle=0.0
        disk = 0.0
        link1 = 0.0
        link2 = 0.0
        link3 = 0.0
        link4 = 0.0
        claw = 0.0
        grab = 0.0 
        x = 0.0
        y = 0.0
        z = 0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                    disk = 0.0                    
                    link1 = 0.0
                    link2 = 0.0
                    link3 = 0.0
                    link4 = 0.0 
                    claw = 0.0
                    grab = 0.0 
                    x = 0.0
                    y =0.0 
                    z = 0.0
                    rotation = 0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE

                elif key == 'i':
                    link1 += MOVE_LINK1
                elif key == 'k':
                    link1 -= MOVE_LINK1

                elif key == '8':
                    link2 += MOVE_LINK2
                elif key == '2':
                    link2 -= MOVE_LINK2

                elif key == '4':
                    link3 += MOVE_LINK3
                elif key == '6':
                    link3 -= MOVE_LINK3

                elif key == 'h' :
                    link4 += MOVE_LINK4
                elif key == 'b' :
                    link4 -= MOVE_LINK4  


                elif key == 'r':
                    disk += DISK_MOVEMENT  #bottom disk rotation
                elif key == 'f':
                    disk -= DISK_MOVEMENT    


                elif key == 'c':
                    grab += CLAW_ANGLE
                elif key == 'o':
                    grab -= CLAW_ANGLE


                # elif key == 'x':
                #     grab -= CLAW_ANGLE
                # elif key == 'y':
                #     rotation += DISK_MOVEMENT
                # elif key == 'z':
                #     rotation += DISK_MOVEMENT
                

                if steer_angle>1.0:
                    steer_angle=1.0
                if steer_angle<-1.0:
                    steer_angle=-1.0

                print("Steer Angle: ",steer_angle)
                print("Linear Velocity: ",linear_vel)
                print("Disk angle: ",disk)
                print("Link_1 angle: ",link1)
                print("Link_2 angle ",link2)
                print("Link_3 angle ",link3)
                print("Link_4 angle ",link4)
                # print("man rotation: ",rotation)

                # Publish the twist message
                wheel_velocities.data = [-linear_vel,-linear_vel,-linear_vel,-linear_vel]
                joint_positions.data = [steer_angle,steer_angle,arr[0],arr[1],arr[2],arr[3],arr[4],-grab,grab]
                # arm_positions.data =[rot,rot,rot,rot,rot,rot,rot]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)
                # self.arm_positions_pub.publish(arm_positions)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import sympy as sp

# # Define symbols for joint variables
# q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')

# # Define forward kinematics equations for each joint
# # This involves defining transformation matrices for each joint and multiplying them
# # to get the transformation matrix from the base to the end effector

# # Define link lengths and DH parameters (this depends on your manipulator's design)
# # Here, a, alpha, d, and theta represent the DH parameters
# # Define your DH parameters accordingly
# a = [0, 0.4, 0.4, 0.4, a4, a5]  # Link lengths
# alpha = [sp.pi/2, 0, 0, sp.pi/2, -sp.pi/2, 0]  # Twist angles
# d = [d1, 0, 0, d4, 0, d6]  # Offsets
# theta = [q1, q2, q3, q4, q5, q6]  # Joint angles

# # Define transformation matrices
# T = []
# for i in range(6):
#     # Define the transformation matrix for each joint
#     Ti = sp.Matrix([[sp.cos(theta[i]), -sp.sin(theta[i]) * sp.cos(alpha[i]), sp.sin(theta[i]) * sp.sin(alpha[i]), a[i] * sp.cos(theta[i])],
#                     [sp.sin(theta[i]), sp.cos(theta[i]) * sp.cos(alpha[i]), -sp.cos(theta[i]) * sp.sin(alpha[i]), a[i] * sp.sin(theta[i])],
#                     [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]],
#                     [0, 0, 0, 1]])
#     T.append(Ti)

# # Define the overall transformation matrix from base to end effector
# T0_6 = sp.simplify(T[0] * T[1] * T[2] * T[3] * T[4] * T[5])

# # Define the end effector position in terms of the joint angles
# end_effector_position = T0_6[:3, 3]

# # Given x, y, z coordinates, substitute these values into the position equation
# x, y, z = 0.5, 0.5, 0.5  # Replace these with your desired coordinates
# end_effector_position_sub = end_effector_position.subs({d1: x, d4: y, d6: z})

# # Solve for joint angles using inverse kinematics
# # You might need to specify initial guesses for the joint angles
# joint_angles = sp.nsolve(end_effector_position_sub, (q1, q2, q3, q4, q5, q6), (0, 0, 0, 0, 0, 0))

# print("Joint angles:", joint_angles)









#################
# q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4')
# x, y, z = sp.symbols('x y z')
#     # DH parameters
# a = [0, 0.4, 0.4, 0.4]  # Link lengths
# alpha = [sp.pi/2, 0, 0, 0]  # Twist angles
# d = [0.03, 0, 0, 0]  # Offsets

# # Define transformation matrices for each joint
# def dh_transform(a, alpha, d, theta):
#     T = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, a],
#                    [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha), -d * sp.sin(alpha)],
#                    [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha), d * sp.cos(alpha)],
#                    [0, 0, 0, 1]])
#     return T

# # Compute individual transformation matrices
# T0_1 = dh_transform(a[0], alpha[0], d[0], q1)
# T1_2 = dh_transform(a[1], alpha[1], d[1], q2)
# T2_3 = dh_transform(a[2], alpha[2], d[2], q3)
# T3_4 = dh_transform(a[3], alpha[3], d[3], q4)

# # Define the overall transformation matrix from the base to the end-effector
# T0_4 = T0_1 * T1_2 * T2_3 * T3_4

# # Extract the translation part of the matrix
# end_effector_position = T0_4[:3, 3]

# # Equate end effector position to the desired x, y, z values
# desired_position = sp.Matrix([x, y, z])
# equations = sp.Eq(end_effector_position, desired_position)

# # Solve for joint angles
# solution = sp.solve(equations, (q1, q2, q3, q4))
# print(solution)
