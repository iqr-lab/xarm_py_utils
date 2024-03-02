# Installation
Clone this package into your ROS workspace:
```
cd ~/dev_ws/src
git clone git@github.com:iqr-lab/xarm_py_utils.git
```
Build your workspace:
```
cd ~/dev_ws
colcon build
```
Source your workspace:
```
source ~/.bashrc
```

# Running a policy with moveit_servo
Launch Gazebo and RViz:
```
ros2 launch xarm_planner xarm7_planner_gazebo.launch.py add_gripper:=true
```

In another terminal, launch the servo controller:
```
cd ~/dev_ws/src/xarm_py_utils/launch
ros2 launch servo.launch.py
```

The servo controller listens for commands on the ```/servo_server/delta_twist_cmds``` rostopic. We'll now publish to that topic based on the output of our policy.
In yet another terminal, launch the policy node:
```
cd ~/dev_ws/src/xarm_py_utils/xarm_py_utils
ros2 run xarm_py_utils policy
```
You should see the robot move! This sample policy (defined in ```xarm_py_utils/policy.py```) moves the robot's gripper in a box-like shape. If you don't see the robot moving, it may be in/near a [singularity pose](https://www.universal-robots.com/articles/ur/application-installation/what-is-a-singularity/). Check the servo controller's terminal for a message like this:
```
Moving closer to a singularity, decelerating
```
If you see this message, try:
* Quitting the policy node
* Using RViz to control the robot arm to a new configuration
* Restarting the policy node
