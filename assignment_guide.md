# Figure 8 HomeR

## 1. Overview
Imagine the scenario that an aerial drone and a ground rover are tracking a shared target.
The success of such a cooperative mission depends on a unified understanding of space.
Each participant sees the world from their own perspective:
- The Drone sees the target relative to its downward-facing camera.
- The Rover sees the target relative to its extended sensor mast.
- The Mission Controller needs to know where both are relative to a fixed map or the Earth.

We can "attach" coordinate frames to the robots' bodies, to the target, to the sensors, to the Earth, etc. to easily "translate" between these perspectives.
This is where `tf2` (a.k.a. Transform Version 2) comes in.

In ROS 2, coordinate frames are organized into a tree structure. Every frame has one "parent" and can have multiple "children".
By following the "branches" of this tree, `tf2` can mathematically chain transforms together. 
It allows us to manage multiple coordinate frames over time and answer the fundamental question: “**What is the pose of object A in the coordinate system of frame B?**” (e.g. What is the target's position relative to the rover).

In this assignment, we will practice frame transforms using `tf2` by achieving the following goals:
1. Setup a (fixed) global frame: `odom` and a body frame attached to HomeR: `base_link`.
2. Broadcast `base_link`'s position and orientation relative to `odom` frame, given the measure velocity of HomeR.
3. Feed HomeR's measured velocity as the input velocity commands to simulate HomeR's motion using `turtlesim`. 
 
## Get Started
- Have HomeR or your favorite ground robot assembled.
- Let the [Pico Messenger](https://github.com/linzhanguca/homer_pico) running at the background on Pico.
So that HomeR's linear and angular velocity can be set and measured from a Raspberry Pi.
- If `turtlesim_node` is running on a different computer other than Raspberry Pi, make sure `ROS_DOMAIN_ID` on these computers are the same.
Attach `export ROS_DOMAIN_ID=<numer>` to the end of `~/.bashrc` file **on both computers**, then `source ~/.bashrc`.
- Review the trajectory calculation from robotics 1's [slides](https://linzhanguca.github.io/_docs/robotics1-2025/1014/kinematics.pdf).
If you are running out of time, HomeR's [hardware interface node](https://github.com/linzhangUCA/homer_bringup/blob/main/homer_bringup/homer_interface.py) is the goto.
- You can build your own package, but one has been prepared in this repository.
###

## Requirements: 
1. (85%) Use [odom_talker.py](homer8_odom_pkg/homer8_odom_pkg/odom_talker.py).
   Fill code between the commented lines:
   ```python
   ### START CODING HERE ###

   ### END CODING HERE ###
   ```
   - (15%) Correctly initialize:
     - a `/odom` topic publisher
     - a tf broadcaster for transform between `odom` frame and `base_link` frame.
     - a timer running at 50 Hz with `announce_odometry` to be its callback function.
   - (10%) Correctly compute the HomeR's pose at each instance.
   - (30%) Correctly format `Odometry` message and **publish** it under the `/odom`.
   - (30%) Correctly format `TransformStamped` message and **broadcast** transformation from `odom` frame to `base_link` frame.
   
2. (10%) Take a screenshot with the turtlesim canvas showing turtle1's trajectory and Rviz world displaying `odom` and `base_link` frames. 
   upload your screenshot to the [images/](images/) directory.
   Name the file [odom_screenshot.png](images/odom_screenshot.png).
   
3. (5%) Fill the `<description>`, `<maintainer>`, `<maintainer_email>` fields with your own information in [package.xml](homer8_odom_pkg/package.xml) and [setup.py](homer8_odom_pkg/setup.py).
Look for the fields marked with `TODO` in these files.


- Download and build the assignment package. 
   1. (Optional) [Create a ROS workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#create-a-new-directory). 
   2. Clone this repository down to the `/src` dirctory in your ROS workspace, then `colcon build`.
      1. Open a terminal window and run following commands:
         ```console
         cd <ros workspace path>
         colcon build --packages-select homer8_odom_pkg
         source install/local_setup.bash  # CRITICAL, or ROS can't find your package
         ```
      2. Start the `turtlesim`
         ```console
         ros2 run turtlesim turtlesim_node
         ```
      3. Run executable `paint_8` in another terminal
         ```console
         source <ros workspace path>/install/local_setup.bash
         ros2 run homer8_odom_pkg paint_8
         ```
         > The robot will wait 5 seconds before moving.

  ![homer8_demo](/images/homer8_demo.gif)

  The velocity commands to drive the robot is predefined in [homer_figure8.py](homer8_odom_pkg/homer8_odom_pkg/homer_figure8.py).
  The robot is supposed to paint a figure 8 on its movable plane, but you'll observe the deviance between theory and reality.


### Hints
- To calculate updated pose for the robot, please review [Assignment 3](https://classroom.github.com/a/R9LNWs9-).

- Please refer to the [official tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html) or the [in-class example](https://github.com/linzhangUCA/4421example_tf) to get a better idea on how to broadcast transformations in ROS.
   
- `turtlesim` is used to visualize the robot's trajectory in this assignment.
  We'll read the robot's actual velocity from the Pico, fill it to the `Twist` message then publish to`/turtle1/cmd_vel` topic.
  Check [homer_figure8.py](homer8_odom_pkg/homer8_odom_pkg/homer_figure8.py) for a detailed management.

- The robot's real-time actual velocity is stored in `self.real_lin_vel` and `self.real_ang_vel`.
  They are extracted in the `talk_listen_pico()` function in [homer_figure8.py](homer8_odom_pkg/homer8_odom_pkg/homer_figure8.py)
  ```python
    if self.pico_messenger.inWaiting() > 0:
      real_vels = (
          self.pico_messenger.readline().decode("utf-8").rstrip().split(",")
      )
      if len(real_vels) == 2:
          try:
              (self.real_lin_vel, self.real_ang_vel) = tuple(
                  map(float, real_vels)
              )
          except ValueError:
              pass
  ```

- Use **Rviz** to verify your odometry setup
  1. Start `rviz2` in terminal
  2. In "Displays" panel, change "Fixed Frame" to **odom**.
  3. Add "TF" to the "Displays" panel, and "Show Names"

## Turtle and Frames

![homer8_screenshot](images/odom_screenshot.png)

## AI Policies
Please acknowledge AI's contributions according to the policies in the [syllabus](https://linzhanguca.github.io/_docs/robotics2-2025/syllabus.pdf).
