# Custom Cyton Arm Setup

### Instructions for working with the arm

1. Build the catkin directory
```
catkin_make && source devel/setup.sh
```
On success: 

2. Bring up robot and execute simple movement
```
roslaunch cyton_control robot_manipulation_real.launch
```

On success: 

3. Execute simple movement task (on a new sourced terminal)
```
rosrun cyton_control cyton_control_node
```

The code for the control of the robot, and the associated path planning algorithms (simple sinusoidal motion) are defined on the **cyton_control** 
package. Please modify based on the application.