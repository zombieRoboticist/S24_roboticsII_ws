# S24_roboticsII
ROS2 Workspace for S24 RoboticsII

## Cloning the repository 
On the robot, open a new terminal (via VNC or SSH)
```
cd ~/codes
git clone https://github.com/eric565648/S24_roboticsII_ws.git [team_name]_ws
```

## Building the ROS2 workspace
The step will build and ROS2 workspace and compile the packages.

**Important**: You have to compile the package **EVERYTIME** you modify the code, evem the code is written in **PYTHON**

Open/access a docker container via a terminal (VNC or SSH)
```
cd ~/codes/[team_name]_ws
colcon build
```
or simply build one specific package
```
colcon build --packages-select [package_name]
```
The [package_name] are for example, `object_detection`, `tracking_control`.

## Activate ROS2 environment
Activate ROS2 environment to run ROS software

**Important**: You have to activate ROS2 environment **EVERYTIME** you open a new terminal and run/access the docker container.

The reason you don't need to do this while doing previous mini project is that there's an ROS activation command in `/root/.bashrc` in the docker container. Whenever a terminal is launched, it will exected this bash script once.

Open/access a docker container via a terminal (VNC or SSH)
```
cd ~/codes/[team_name]_ws
source install/setup.bash
```

## Launch tracking nodes!
Open a terminal and access docker (via VNC or SSH). Remeber to **Activate ROS2 environment**.
