# Anti-Kaboom-

## 1. Introduction

In military operations, the critical task of diffusing explosives demands a blend of precision, safety, and technological advancement. Our focus centers on a specialized robot—a robust four-wheeled vehicle complemented by a five-degree-of-freedom (5 DOF) manipulator arm. Designed to be deployed in military environments, the robot stands equipped with large wheels for navigating challenging terrains and a manipulator arm with a claw-like end-effector.

The robot model is designed using SolidWorks (CAD software), and then the kinematics are simulated in Gazebo using ROS2. We utilize the Denavit-Hartenberg (D-H) parameters to define the manipulator arms kinematic properties. We delve into the mathematical formulations of forward and inverse kinematics, explaining the methodologies employed to determine both end-effector position and manipulator arm configurations. Additionally, this report presents the validation processes for both forward and inverse kinematics, outlining the techniques utilized to verify the accuracy and efficiency of these fundamental calculations.

## 2. Application

### Military Use:

1. **Safety & Precision:**
   - Handling Explosives: The primary purpose is to handle and disarm explosive devices from a safe distance, minimizing risk to human life.
   - Remote Operations: Allows for remote handling, reducing direct human exposure to dangerous situations.

2. **Terrain Versatility:**
   - Off-road Navigation: Large off-road wheels enable the robot to traverse rough, uneven, or challenging terrains common in military or disaster-stricken areas.
   - Accessibility: Provides access to remote or difficult-to-reach locations where explosive devices might be hidden or placed.

3. **Manipulation Capabilities:**
   - Manipulator Arm: The 5 DOF arm offers a wide range of motion and precision, enabling intricate manipulation to defuse or handle explosive devices with specialized tools like claws.

### Civilian & Disaster Response:

1. **Public Safety:**
   - Law Enforcement & Emergency Response: Useful for law enforcement, bomb squads, or emergency responders in handling improvised explosive devices (IEDs) or unexploded ordnance.
   - Search & Rescue: Can access hazardous areas in disaster zones to identify and neutralize explosive threats that may obstruct rescue efforts.

2. **Remote Operations:**
   - Risk Mitigation: Reduces the risk to human lives by executing dangerous tasks remotely.
   - Precision Handling: Offers a controlled and precise approach to handling explosive materials without risking human error.

## 3. Robot Type

The Anti-Kaboom is a wheeled robot with a 5 degree of freedom arm in the middle of the chassis, giving it the ability to reach objects 360 degrees around the robot while maintaining stability for the robot as the arm is at the center of mass of the chassis.

## 4. Degrees of Freedom (DOFs) and Dimensions

The manipulator arm has 5 degrees of freedom. The base rotates 360 degrees about the z-axis, allowing the arm to reach around the chassis. The 3 leg links allow the end effector to reach a desired location by rotating about the y-axis, and the end effector leg gives more precision when aligning the claws when picking up an object.

The 4-wheeled car chassis has 3 degrees of freedom:
- Translation along the x-axis: Forward and backward movement.
- Translation along the y-axis: Side-to-side movement or lateral movement.
- Rotation about the z-axis: Turning or steering.

The wheels rotate about an axis, allowing them to move forward or backward, and the front two wheels rotate about the z-axis to steer the car.

## CAD Models

[Link to CAD Models](#) <!-- Replace with the actual link to your CAD Models -->

## Cloning the Repository and Installing Packages

To clone the repository and install the relevant packages for Anti-Kaboom, follow these steps:

1. Open your terminal.
2. Run the following command to clone the repository:
   ```bash
   git clone https://github.com/Suraj-Kalwaghe/Anti-Kaboom.git
   ```

3. Navigate to the cloned directory:
   ```bash
   cd anti-kaboom
   ```

4. Install ROS2 dependencies and packages:
   ```bash
   # Example command, adjust based on your ROS2 setup
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the workspace:
   ```bash
   colcon build
   ```

## Launching the Model in Gazebo

To launch the model in Gazebo using the provided `gazebo.launch.py` file, use the following command:

```bash
ros2 launch anti_kaboom gazebo.launch.py
```

This launch file initializes Gazebo with the Anti-Kaboom model, allowing you to visualize and simulate its movements.

**Note:** Ensure that you have Gazebo and ROS2 properly installed on your system before running the launch command.
Feel free to explore other launch files and scripts provided in the repository for additional functionalities.
Enjoy exploring the capabilities of the Anti-Kaboom mobile robot!
