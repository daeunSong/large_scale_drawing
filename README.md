# large-scale-drawing
Large-scale robotic drawing


```sh
roslaunch ridgeback_iiwa_gazebo ridgeback_iiwa_gazebo.launch
roslaunch large_scale_drawing drawing.launch
```

sme as


```sh
roslaunch ridgeback_iiwa_gazebo ridgeback_iiwa_gazebo.launch
roslaunch ridgeback_navigation odom_navigation_demo.launch
roslaunch ridgeback_iiwa_moveit move_group.launch
roslaunch ridgeback_iiwa_viz view_robot.launch config:=drawing
rosrun large_scale_drawing drawing_moveit
```
