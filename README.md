# large-scale-drawing

*Tested on **Ubuntu 18.04** with **ROS Melodic**.*

<img src="./doc/img/demo.gif" width="600">

## Build and Compile

1. Clone this repository:
```sh
mkdir ros_ws && cd ros_ws && mkdir src
catkin_init_workspace
cd src
git clone https://github.com/daeunSong/large_scale_drawing.git
```

2. Clone **iiwa** and **ridgeback** related repositories:
```sh
git clone -b glab/drawing https://github.com/daeunSong/iiwa_stack.git
git clone -b glab/integration git clone https://github.com/ridgeback/ridgeback.git
git clone https://github.com/ridgeback/ridgeback_desktop.git
git clone https://github.com/ridgeback/ridgeback_simulator.git
git clone https://github.com/daeunSong/ridgeback_iiwa_integration.git
```

3. Install the dependencies:
```sh
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

4. Add a following line in ~/.bashrc:

`export RIDGEBACK_URDF_EXTRAS=$(catkin_find ridgeback_iiwa_description urdf/ridgeback_iiwa_robot.urdf.xacro --first-only)`


5. Build the workspace:
```sh
catkin build
```

6. Source the workspace:
```sh
source devel/setup.bash
```

## Demo

1. Run gazebo and bring up the robot model
```sh
roslaunch ridgeback_iiwa_gazebo ridgeback_iiwa_gazebo.launch world_name:=empty
```

2. Run moveIt! and rviz visualization related nodes
```sh
roslaunch large_scale_drawing prep_drawing_moveit.launch
```

3. Run robot drawing
```sh
roslaunch large_scale_drawing drawing_manager.launch
```

To change the drawing input, change the name of the configuration file in [here](https://github.com/daeunSong/large_scale_drawing/blob/31b85f34acbd624ab041da2da8223dcf6439c6a2/iiwa/launch/prep_drawing_moveit.launch#L8). Please refer to the details of the **config file** in [here](https://github.com/daeunSong/large_scale_drawing/tree/debug/data/config) and the **input drawing** file in [here](https://github.com/daeunSong/large_scale_drawing/tree/debug/data/input).

