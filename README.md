# ur10e_controller

Robot control, realtime reconstruction, and defect detection for AI4CE lab M3DP project.

Authur: [Yongyu Liu](https://github.com/YongyuLiu03)  [yl8126@nyu.edu](mailto:yl8126nyu.edu)

# Tutorial

## Hardware connection and basic setup

_For more information, check official repository [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)_

1. Ensure that the computer has enabled real-time capablities, run `uname -a` and find `PREEMPT_RT`. This is necessary for stable connection. 

   ![Screenshot from 2023-08-05 16-01-16](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/d319fa49-3c1b-4eb0-904a-56a6516fa3e7)

   ![Screenshot from 2023-08-05 16-12-55](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/c16444a5-9c3c-4afe-9d71-fb334226f99c)

2. Wired connect computer with the robot arm, in computer network settings, find the wired connection and set it as in the picture.

   ![Screenshot from 2023-08-05 15-52-48](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/5dcec0b7-9321-4304-a4e8-13611aaa6806)

3. On robot arm's teaching pendant, find Settings -> System -> Network and set it.

   ![IMG_20230805_155743](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/347f400b-2478-43a8-9e7e-a8d8e72473b8)
  
4. Start `roscore`

5. In robot's NORMAL mode, load an external control program

  ![IMG_20230805_160555](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/7a460f65-4cbc-4099-b316-24d6e14821ae)

6. Start a ROS node that contains `$(find ur_robot_driver)/launch/ur10e_bringup.launch`

7. Start the loaded program on the teaching pendant.
   
  ![IMG_20230805_160605](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/15b43b79-b93a-471c-ae6e-17d0fa3dce6f)

  You should see the following message in the driver node
  
  ![Screenshot from 2023-08-05 16-06-23](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/11dd6979-37cc-42dc-8623-91f0856149c9)

## Hand-eye calibration

_For more information, check [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)_

1. Run `roslaunch ur10e_controller handeye_calibration.launch`, once 3 windows pop up, the errors can be safely ignored.

2. Run `rqt`. In Plugins -> Visualization -> Image view, subscribe topic `/aruco_tracker/result`. You should see these windows. 

  ![Screenshot from 2023-08-05 16-31-21](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/76f7578a-e872-462a-b2a0-819e50933c47)

3. Only take samples when the result shown in rqt is correct and stable. The calibration result will be stored at `$HOME/.ros/easy_handeye/ur10e_calib_eye_on_hand.yaml`

## Experiment

1. Run `roslaunch ur10e_controller bringup_all.launch`.

2. Run `roslaunch ur10e_controller publish_calibration.launch`, check that frame `camera_color_optical_frame` has appeared in RViz.
   
  ![Screenshot from 2023-08-05 16-41-47](https://github.com/YongyuLiu03/ur10e_controller/assets/83950768/ca5cd6a9-c6a4-4f54-813f-a6e8a14f1494)

3. Run `rosrun ur10e_controller realtime.py`, a window displaying real-time point clouds after denoise and reconstruction will pop up.

   Whenever a message is published in the topic `/height`, by printing program or via terminal, e.g. `rostopic pub /height std_msgs/Float64 0.04`, the window will make a comparison between current scene and model cropped in z-axis by corresponding height.
   
4. Run `rosrun ur10e_controller print_reconstruct.py`, the robot will compute and store plans for printing a cubic, then waits input in terminal to execute the plans.

   Or run `rosrun ur10e_controller direct_print.py`, which loads pre-computed plans and execute them.

## References
- [Open3D](http://www.open3d.org/)
- [IntelRealSense](https://github.com/IntelRealSense)
- [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
- [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [universal_robot](https://github.com/ros-industrial/universal_robot)
