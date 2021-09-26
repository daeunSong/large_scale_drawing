# large-scale-drawing
Large-scale robotic drawing

Clone the *moveit_visual_tools* repository of ours for better visualization result.
```shell
git clone -b glab/drawing https://github.com/daeunSong/moveit_visual_tools.git
```

```sh
roslaunch ridgeback_iiwa_gazebo ridgeback_iiwa_gazebo.launch
roslaunch large_scale_drawing drawing.launch
rosrun large_scale_drawing line_publisher
```

same as


```sh
roslaunch ridgeback_iiwa_gazebo ridgeback_iiwa_gazebo.launch
roslaunch ridgeback_navigation odom_navigation_demo.launch
roslaunch ridgeback_iiwa_moveit move_group.launch
roslaunch ridgeback_iiwa_viz view_robot.launch config:=drawing
rosrun large_scale_drawing drawing_moveit
rosrun large_scale_drawing line_publisher
```
