# Generic Robot Design, Simulation and Control

![](/Mechanics/V2/Untitled3.png)

This program provides functionalities for controlling robotic arms in simulation environments. It allows you to simulate, model, and control robot movements for various applications.

## Installation

To run this program, you need Python installed on your system. Additionally, make sure you have the following dependencies installed:

- `numpy`
- `time`
- `pybullet`
- `matplotlib`
- `GsUsb`

## Usage

The program offers different functionalities for robot control, which are described below:

### 1. Using a PyBullet Robot (Simulation)

To use a PyBullet robot, follow these steps:

```python
# Uncomment the following lines to use a PyBullet robot
# robot = Robot()
# robot.launch_URDF_simulation("kuka_iiwa/model.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.view_mode()
```

### 2. Design Your Own Robot Using DHM Parameters (Simulation)

You can design your own robot using DHM parameters. Here's an example for a 4R robot:

```python
# Example: 4R Robot
# robot = Robot(joint_types=[REVOLUTE, REVOLUTE, REVOLUTE, REVOLUTE],
#               a=[0, 2, 1.5, 1],
#               alpha=[np.pi/2, 0, 0, 0],
#               d=[1, 0, 0, 0],
#               theta=[np.pi/2, 0, 0, 0])
# robot.create_simplified_URDF_based_on_DHM_parameters()
# robot.launch_URDF_simulation("outfile.urdf", startPos=[0,0,0.2], fixedBase=True)
# robot.simulate.view_mode()
```

### 3. Verify Your Modelisation (Simulation)

You can verify your model by visualizing joint and link frames:

```python
# Uncomment and modify as needed
# robot = Robot()
# robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.set_endEffector_frame_offset(0.0,0.0,-0.111)
# robot.simulate.view_all_joint_frames()
# robot.simulate.view_all_link_frames()
# robot.simulate.view_endEffector_frame()
# robot.simulate.view_mode()
```

### 4. Position Joint Control (Simulation)

Control the robot's joints to achieve desired positions:

```python
# Uncomment and modify as needed
# robot = Robot(joint_distances=[0.2,0.15,0.127,0.111])
# robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.move_joint(joint_index=1, displacement=0.2555, max_speed=0.2, torque=100, acceptable_error=1E-5, wait_to_exit=True, plot_joint_positions=False)
# Add more move_joint commands as required
```

### 5. Position Robot Control (Simulation) Using Your Modelisation

Control the robot's position using your model:

```python
# Uncomment and modify as needed
robot = Robot(joint_distances=[0.2,0.15,0.127,0.111])
robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
# Add move_robot_position commands as required
```

### 6. Trajectory Robot Control (Simulation)

Control the robot's trajectory:

```python
# Uncomment and modify as needed
# robot = Robot()
# robot.launch_URDF_simulation("./scara/urdf/scara.urdf", startPos=[0,0,0], fixedBase=True)
# robot.move_robot_moveit_trajectory("./Tools/trajectory.txt")
```

### 7. Real Robot Control

This part is currently under development and not ready for use.
