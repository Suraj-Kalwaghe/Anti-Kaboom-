#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import math
from matplotlib import pyplot as plt


#Data Log for report
TimeVector = []
Ctrl_Inputs = []
Error_Vector = []

# Start P controller
class P_Controller(Node):
    def __init__(self):
        super().__init__('p_controller_node')

        #publisher to controll car velocities and turning
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        
        #subcriber to IMU
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)

        #ttime call back
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #init 
        self.current_heading = 0.0
        self.timestamp = 0.0        # time for report
        self.target_x = 10          # Target X position
        self.target_y = 10          # Target Y position
        self.current_x = 0.001      # Current X position
        self.current_y = 0.001      # Current Y position
        self.linear_vel = 5.0       # Velocity
        self.steer_angle =0.0       # Turning
        self.Wheelbase = 20         # wheelbase to calculate wheel turning angle
        self.phi_error = 0.0        # phi error
        self.slow_stop = 0.1        # distance to start slow down 

        # get info from IMU
    def imu_callback(self, msg):
        current_quaternion = msg.orientation    # get quaternion (x,y,z,w)
        self.current_heading = self.quaternion_to_yaw(current_quaternion) # get actual direction from quaternion to rad
        self.timestamp = msg.header.stamp.sec   # get IMU time for report
        
        # calculate quaternion to rad
    def quaternion_to_yaw(self, quaternion):
        yaw = 2.0 * math.atan2(quaternion.z, quaternion.w) # this funcation get correct yaw.
        # yaw = 2.0 * math.atan2(2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x), quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z) #for (x,y,z,w)
        # yaw = 2.0 * math.atan2( 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)) # for (w,x,y,z)
        return yaw  #out put yaw in rad
        
        #positoin calculater
    def update_position(self):
         # Calculate the distance traveled based on time_interval *** base only front wheel turning car
        delta_x = self.linear_vel * math.cos(self.current_heading+self.linear_vel/self.Wheelbase*math.tan(self.steer_angle)) * 0.01
        delta_y = self.linear_vel * math.sin(self.current_heading+self.linear_vel/self.Wheelbase*math.tan(self.steer_angle)) * 0.01
        # Update the current position
        self.current_x += delta_x
        self.current_y += delta_y


        # move to target
    def move_to_position(self):

        # init publish info (velocity and turning)
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()

        # target direction (rad)
        angle_to_target = math.atan2(self.target_y-self.current_y, self.target_x-self.current_x)

        # distance to the target (x)
        distance_to_target = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)



        if distance_to_target > self.slow_stop:

            # Keep a high linear velocity until approaching the slow_down_distance
            if distance_to_target > self.slow_stop + 1.2:
                self.linear_vel = self.linear_vel  # High linear velocity
                
                # Calculate the error between current heading and the angle to the target
                self.phi_error = angle_to_target - self.current_heading
                # Use a proportional controller to adjust the steering angle
                Kp = 0.3  # Proportional Controller Gain
                desired_angle = -Kp * self.phi_error
                self.steer_angle = math.atan(self.Wheelbase * math.tan(desired_angle) / (self.linear_vel))
                # Implement joint limits for the steer angle
                if self.steer_angle>1.0:
                    self.steer_angle=1.0
                if self.steer_angle<-1.0:
                    self.steer_angle=-1.0
            else:
                # linear reduce velocity as you get closer to the target
                self.linear_vel = max(self.linear_vel - 0.05, 1)  # Adjust the deceleration rate
        else:
            # Vehicle has reached the target, stop completely
            self.linear_vel = 0.0
            wheel_velocities.data = [self.linear_vel, -self.linear_vel]
            self.wheel_velocities_pub.publish(wheel_velocities) 
            rclpy.shutdown() 

        # get array data for publish
        wheel_velocities.data = [self.linear_vel, -self.linear_vel]
        joint_positions.data = [-self.steer_angle, self.steer_angle]

        #log data for report
        TimeVector.append(self.timestamp)
        Ctrl_Inputs.append(-self.steer_angle*180.0/math.pi)
        Error_Vector.append(self.phi_error*180.0/math.pi)

        # Publish joint and velocity commands
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        
        # Terminal output, to check any error
        self.get_logger().info(f'v{self.linear_vel}Target{angle_to_target*180.0/math.pi:.1f}da{self.phi_error*180.0/math.pi:.2f}a{self.steer_angle*180.0/math.pi:.2f} , Position: {self.current_x:.2f}, {self.current_y:.2f}')
        # self.get_logger().info(f'{self.current_heading*180.0/math.pi:.2f}{self.phi_error*180.0/math.pi:.2f}da{self.steer_angle*180.0/math.pi:.2f}T{self.timestamp}Heading: {self.current_heading*180/math.pi:.2f}')
        # self.get_logger().info(f'{self.current_heading*180.0/math.pi:.2f}T{self.timestamp}')

        

    def timer_callback(self):
        self.move_to_position()
        self.update_position()


def main(args=None):
    rclpy.init(args=args)
    Controller = P_Controller()

    try:
        rclpy.spin(Controller)
    except KeyboardInterrupt:
        pass

    Controller.destroy_node()
    rclpy.shutdown()


    # draw yaw error and steer angle vs. Time graph
    plt.figure(1)
    plt.title('Yaw Error and Steer Angle vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.plot(TimeVector, Error_Vector, 'b-', label='Yaw Error')
    plt.plot(TimeVector, Ctrl_Inputs, 'r-', label='Steer Angle')
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
