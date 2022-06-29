# rosbag Recoding and Usage

We record a robot state using rosbag in order to analyze the drawing result.

## Recording

1. Open a terminal and write the commands below:
    ```shell
    roscd large_scale_drawing/data/bag
    rosbag record -O drawing /iiwa/state/CartesianPose
    ```

2. When the recording is done, press `ctrl + c` in the terminal. This will save a `drawing.bag` file.


3. Copy and paste the Start and End timestamp from the terminal into `time_stamp.txt`. 

## Reading

1. Read the rosbag file and save the points in a text file.
   ```shell
   rosrun large_scale_drawing read_bag
   ```
   This will save the robot trajectories in `drawing_traj.txt` stroke by stroke. 


2. Plot the result. 
   ```shell
   python3 -i plot_traj.py
   ```
