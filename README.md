# large-scale-drawing
Large-scale robotic drawing

```shell
git clone -b glab https://github.com/daeunSong/vive_tracker.git
```

```sh
roslaunch viv_tracker vive_tracker.launch
roslaunch large_scale_drawing 

roslaunch ridgeback_iiwa_gazebo ridgeback_iiwa_gazebo.launch
roslaunch large_scale_drawing prep_drawing_moveit.launch
roslaunch large_scale_drawing drawing_manager.launch
```

