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

![homer8_demo](/images/homer8_demo.gif)


## 2. Get Started
To simulate a physical robot's movement, we can measure the velocity of the robot using on-board sensors (e.g. encoders).
Then feed the measured velocity as the target/reference velocity command for the differential drive turtle from the `turtlesim_node`.

### 2.1. Reference Workflow
1. Open a terminal and start `turtlesim_node`
```console
ros2 run turtlesim turtlesim_node
```
2. Start transform broadcasting node
```console
ros2 run <homer8_package> <tf_bc_node> 
```
3. Start robot driving node
```console
ros2 run <homer8_package> <driver_node> 
```

> [!NOTE]
> The commands wrapped in `<>` are customizable.

### 2.2 Preparation
- Have HomeR or your favorite ground robot [assembled](https://linzhanguca.github.io/homer_docs/mechanical/assembly/).
- Let the [Pico Messenger](https://github.com/linzhanguca/homer_pico) running at the background on Pico.
So that HomeR's linear and angular velocity can be set and measured from a Raspberry Pi.
- If `turtlesim_node` is running on a different computer other than Raspberry Pi, make sure `ROS_DOMAIN_ID` on these computers are the same.
Attach `export ROS_DOMAIN_ID=<numer>` to the end of `~/.bashrc` file **on both computers**, then `source ~/.bashrc`.
- Review the trajectory calculation from robotics 1's [slides](https://linzhanguca.github.io/_docs/robotics1-2025/1014/kinematics.pdf).
If you are running out of time, HomeR's [hardware interface node](https://github.com/linzhangUCA/homer_bringup/blob/main/homer_bringup/homer_interface.py) is the goto.


## 3. Requirements: 

### 3.1. (50%) Broadcast transform
- (5%) Listen to Pico for measured linear and angular velocity of the robot.
- (15%) Publish proper message to the right topic so that the simulated turtle will be driven by the robot's measured velocity.
- (15%) Subscribe to a `Twist` message topic (e.g. `/cmd_vel` from the driver node or `teleop_twist_keyboard`) and transmit `linear.x` and `angular.y` data as the robot's referenced linear and angular velocity to Pico.
- (10%) Compute `base_link` frame's position and orientation relative to the `odom` frame.
Broadcast the tansform of the frames at the frequency of 20 Hz.
- (5%) Illustrate the relationship between `odom` frame and `base_link` frame in a transform tree graph.
Upload the graph to the repository and  

> [!TIP]
> - You can add "TF" in `rviz2` to visualize the transform.
> - You can test tf broadcasting by driving the robot using the keyboard or gamepad.
 
### 3.2. (30%) Drive HomeR in Figure 8s.
- (10%) Create a publisher running at 20 Hz, publish `Twist` message to the right topic to drive the robot making the Figure 8 pattern.
- (20%) The robot needs to traverse two circles at opposite directions, but maintain its angular speed at $\pi/4$ rad/s.
  - The ideal smaller circle is with radius of 1 meter.  
  - The ideal bottom circle is with radius of 2 meters, and the robot should be travelling in opposite direction on it.

> [!TIP]
> - Refer to [Assignment 2](https://classroom.github.com/a/a4Gqehwo).
> - You can either stuff the figure 8 driving to the transform broadcasting node, or create a separate node for it.

### 3.3. (20%) Construct package
- Create a package to host all your executables and make sure `ros2 run <package_name> <executable_name>` command is available after the package is built.
- (10% Bonus) Use one `ros2 launch <package_name> <launch_file_name>` command to launch everything  

### 3.4. (50% BONUS) Ground Demonstration
You can request a live demo to showoff your robot "painting" 8️⃣ figures on the ground.


## AI Policies
Please acknowledge AI's contributions according to the policies in the [syllabus](https://linzhanguca.github.io/_docs/robotics2-2025/syllabus.pdf).
