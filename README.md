# large-scale-drawing
Large-scale robotic drawing in real-world using HTC Vive Tracker.

## Preparation

Our real robot large-scale-drawing make use of HTC Vive tracker to localize the robot and the wall. 
We have modified the script of [this project](https://github.com/moon-wreckers/vive_tracker) to fit our needs. 

Please clone [the repository](https://github.com/daeunSong/vive_tracker.git) and follow the instruction in README:
```shell
git clone -b glab https://github.com/daeunSong/vive_tracker.git
```

## Run
1. Setup the VR devices and run Steam VR

2. Run vive tracker script
```sh
roslaunch vive_tracker vive_tracker.launch
```

3. Calibrate vive tracker with Ridgeback
```sh
rosrun large_scale_drawing vive_tracker_calibration.py
```
Ridegeback will move and calibrate the tracker data with real world.
This only needs to be done once, whenever you change the tracker/light house settings. 

Change the values in [here](https://github.com/daeunSong/large_scale_drawing/blob/1a66ad17f1492e8fb6844f5855976ebc8ada7464/ridgeback/src/vive_tracker_calibration.py#L217) to `move=Ture, debug=True`. This will give you the values needed for calibration. 
Change the values in line [109](https://github.com/daeunSong/large_scale_drawing/blob/1a66ad17f1492e8fb6844f5855976ebc8ada7464/ridgeback/src/vive_tracker_calibration.py#L109), [125, 126](https://github.com/daeunSong/large_scale_drawing/blob/1a66ad17f1492e8fb6844f5855976ebc8ada7464/ridgeback/src/vive_tracker_calibration.py#L125-L126), [137](https://github.com/daeunSong/large_scale_drawing/blob/1a66ad17f1492e8fb6844f5855976ebc8ada7464/ridgeback/src/vive_tracker_calibration.py#L137), [150](https://github.com/daeunSong/large_scale_drawing/blob/1a66ad17f1492e8fb6844f5855976ebc8ada7464/ridgeback/src/vive_tracker_calibration.py#L150), to the values you obtain.

Change the values to `move=False, debug=False`, if you have done all the calibrations and changed the values. Robot will not run, just publish the messgaes.

[//]: # (4. Launch the tf publisher)

[//]: # (```shell)

[//]: # (rosrun large_scale_drawing tf_publisher.py)

[//]: # (```)

4. Launch the preparation launcher for the drawing. Change the `open_rviz` value as your preferation to launch rviz
```shell
roslaunch large_scale_drawing prep_drawing_tracker.launch oepn_rviz:=true
```

6. Run ridgeback and iiwa nodes

```sh
rosrun large_scale_drawing drawing_manager
rosrun large_scale_drawing ridgeback_tracker.py
```

