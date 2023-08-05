# ur10e_controller

Robot control, realtime reconstruction, and defect detection for AI4CE lab M3DP project.

Authur: [Yongyu Liu](https://github.com/YongyuLiu03)  [yl8126@nyu.edu](mailto:yl8126nyu.edu)

# Tutorial

## Hardware connection and basic setup

_For more information, check official repository [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)_

1. Ensure that the computer has enabled real-time capablities, run `uname -a` and find `PREEMPT_RT`. __This is necessary for stable connection.__

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

5. Run `python3 scripts/printhead_control.py` to manually control the printhead motor, or uncomment arduino lines in `print_reconstruct.py` or `direct_print.py` for automatic control and the motor will be stopped when the robot is rotating.

## Troubleshoot

__Do not__ mess with camera's calibration. You can try More -> Calibration Data -> Restore Factory in Intel RealSense Viewer SDK, but __do not__ try other self-calibration methods in SDK unless you have a good reason and are familiar with depth camera calibration. This may result in wrong hand-eye calibration result and erroneous point cloud reconstruction.

For scripts that contain processing point clouds, check the variable `depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()`. Due to unknown hardware issues, the value of depth_scale is either 9.999999747378752e-05 or 0.0010000000474974513. Although they refer to the same length unit, the point cloud will be scaled incorrectly when `depth_scale < 0.001`. The solution is to scale the point cloud down by 0.1 once it is created from rgbd image, e.g. in [realtime.py](https://github.com/YongyuLiu03/ur10e_controller/blob/2a371789d94ba79f7d897c28c006ca2252fdcb7c/scripts/realtime.py#L121). 

For this specific project, use my modified version of these packages instead of the original ones in the catkin workspace: [universal_robot](https://github.com/YongyuLiu03/universal_robot), [Universal_Robots_ROS_Driver](https://github.com/YongyuLiu03/Universal_Robots_ROS_Driver), [realsense-ros](https://github.com/YongyuLiu03/realsense-ros). 

The position of `printer_link` defined in `catkin_ws/src/universal_robot/ur_description/urdf/inc/ur_macro.xacro` may not be accurate and requires manual adjustment. To do this, move the robot's end effector, which is set to be printer_link, to a position. Capture and visualize a frame in Open3D with the printer attached to the robot. In the window, also include a point indicating the goal position. Compare and adjust the translations in `ur_macro.xacro` accordingly, so that the printer's tip and the target point align. This step is necessary because the point cloud comparison relies grealy on the alignment of the printed object and the ideal model.

Due to the unsteadiness of camera holder, tape the camera firmly to the holder before starting the experiment, and recalibrate anytime you feel necessary. 

If the IK solution of robot shown in RViz distorts greatly resulting in collision during movement, the most effetive way is to examine and modify joint degree constraints in `catkin_ws/src/universal_robot/ur_description/config/ur10e/joint_limits.yaml`.

## References
- [Open3D](http://www.open3d.org/)
- [IntelRealSense](https://github.com/IntelRealSense)
- [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)
- [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [universal_robot](https://github.com/ros-industrial/universal_robot)
