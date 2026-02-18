# ROS Odometry
## Objectives
- Practice [transformation broadcasting](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html)
- Review ROS [node](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- Review ROS [topic](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).
- Review ROS [package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) and [executable](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

## Pre-requisite
- Setup motor control interface on the Pico board.
So, the Pico is 
   1. transmitting the robot's actual velocity in real time.
   2. receiving target velocity commands for the robot.
   3. regulate the robot's motion with a PID controller refering to the target velocity commands. 
   
  > Feel free to use the sample scripts from [HomeR](https://github.com/linzhangUCA/homer/tree/2425/homer_control/pico_scripts).

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

## Requirements: 
1. (85%) Complete the [odom_talker.py](homer8_odom_pkg/homer8_odom_pkg/odom_talker.py).
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
