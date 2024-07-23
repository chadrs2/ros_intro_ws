# Introduction to ROS 1 using `turtlesim`

Learning ROS can take time and be frustrating especially without the right resources. So you don't have to go through the pain I did in first learning ROS, here's some helpful tools.

I made a slidedeck that goes over high-level concepts of ROS as well as specific commands that you will often use both in your code scripts as well as in the terminal. Here is a pdf of that [slide deck](https://drive.google.com/file/d/1ot_8GjdilRplizbb9FmRLrovD-lwL25u/view?usp=sharing).

I also made this basic repository that runs through teaching these basic concepts with ROS's `turtlesim`. Again, here's the [github repo](git@github.com:chadrs2/ros_intro_ws.git) to clone.

Here's also ROS's intro to the turtlesim if you want to look at that: https://wiki.ros.org/turtlesim


## Basics

**Install ROS following [these steps](https://wiki.ros.org/noetic/Installation/Ubuntu)!**

ROS always needs a rosmaster to be running before trying to run anyother ROS commands or files. Think like a manager that helps run and connect everything in ROS together (see slides for more details).

There are two ways to start the ROS master:
1. In a terminal run `roscore`
2. When you roslaunch a launch file, it'll start up the rosmaster in the background. We'll go into roslaunch files later on, so just use `roscore` for now.

## Example 1: ROS Graph basics with turtlesim

Steps to display the turtlebot:
1. Source ros environment: 
```
cd ros_intro_ws/
source devel/setup.bash
```
2. Startup the rosmaster: `roscore`
3. Display the turtlesim: `rosrun turtlesim turtlesim_node`
4. In a new terminal, look at the topics:
```
source devel/setup.bash
rostopic list
```

Should show you these topics:
```
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
- `/rosout` and `/rosagg`: Are always there from rosmaster   
- `/turtle1/cmd_vel`: Topic to command the turtle's velocity to move
- `/turtle1/color_sensor`: Topic that simply tells you turtle1's current color in RGB
- `/turtle1/pose`: Topic that tells you the 2D Pose of the turtle.

> Feel free to look into the details of these topics to see what data they expect/use. I normally do the following approach:
> - `rostopic list`: To view available topics
> - `rostopic info <topic_name>`: To see which nodes publish and subscribe to that topic as well as the data type that the topic expects
> - `rosmsg info <msg_type>`: Gives me the details of what the rosmsg is made up of.
> - Then I'll often try publishing data to the topic to see what happens with `rostopic pub -1 <topic_name> <msg_type> '<msg_data>'`. 

5. In this terminal, look the the rosnodes present:
```
rosnode list
```
Should show you these nodes:
```
/rosout
/turtlesim
```
- `/rosout`: Is also always present when rosmaster is running
- `/turtlesim`: The main node that was started up when we ran `rosrun turtlesim turtlesim_node`

6. The best way to visualize all these connections is with `rqt_graph`. Essentially it is a rosnode that shows the entire ros graph network. 
```
rosrun rqt_graph rqt_graph
```
Then change the top-left drop down menu to `Nodes/Topics (all)` and make sure `Dead sinks` and `Leaf topics` are unchecked. You should now see a great rosgraph visual. Note that squares represent topics and ellipses represent nodes. 


## Example 2: Move the bot from the terminal
Let try publishing data over to `cmd_vel` topic to get the turtlebot to move
1. Do steps 1-3 from Example 1
2. Let's first look into the rostopic that moves the turtlebot
```
rostopic list
rostopic info /turtle1/cmd_vel
```
This shows you that only the `/turtlesim` node is subscribed to this topic. So if you publish a command velocity to this topic, then you can assume that turtlesim will grab that data (since it's subscribing to the topic) and likely then move the turtle.

3. Now let's see what data needs to be sent to this topic:
```
rosmsg info /geometry_msgs/Twist
```
Note that since rosmsg are basically like a specific structure/class, we'll see in the next example how you can use them in a python file. For now, we can publish data to this topic from the terminal with the structure of `rostopic pub -1 <topic_name> <msg_type> '<msg_data>'`. In this case an example command could look like:
```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}'
```
Feel free to play around with different values.

## Example 3.1: Publisher - Move the turtlebot from a python rosnode

1. Do step 1 from Example 1
2. `cd src`
3. Create your ros package once in the `src` folder (it will store code from examples 3.1, 3.2, and 4) following the structure of `catkin_create_pkg <pkg_name> <pkd_dependencies>`.
```
catkin_create_pkg turtlebot_pubsub rospy roscpp std_msgs geometry_msgs turtlesim
```
4. Within the newly created ros package, make your python file and make it executable:
```
cd turtlebot_pubsub/src
touch mover.py
chmod +x mover.py
```
5. Write a simple python publisher that publishes velocity commands to the `cmd_vel` rostopic. A publisher turtorial can be found [here](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).
6. Once written, rebuild workspace with
```
cd ~/ros_intro_ws
catkin_make
```
7. Then do steps 1-4 from example 1.
8. To run your own turtlebot mover rosnode, start it with
```
rosrun turtlebot_pubsub mover.py
```
9. You should be seeing the turtle move now! You could also look at the `rqt_graph` to see your new node present.

You can now look at my code in `turtlebot_pubsub/src/mover.py` to compare approaches.


## Example 3.2: Subscriber - Print the pose of the turtlebot

Now we're going to subscribe to the pose of our turtlebot as it changes when we run our `mover.py` node
1. Do steps 1-4 from example 3.1
2. Now write a simple subscriber that listens to the `/turtle1/pose` and prints out the data from this topic.
3. Once written, rebuild workspace with
```
cd ~/ros_intro_ws
catkin_make
```
4. Then do steps 1-4 from example 1 and step 8 from example 3.1.
5. To run your own turtlebot subscriber rosnode, start it with
```
rosrun turtlebot_pubsub watcher.py
```
6. You should be seeing watching the pose change as the turtle is moving! You could also look at the `rqt_graph` to see your new node present.

You can now look at my code in `turtlebot_pubsub/src/watcher.py` to compare approaches.

## Example 4: Roslaunch - Launch both nodes from 3.1 and 3.2

That's a lot of terminals from 3.2. Is there a way to fix that? Yes! Roslaunch was created as a way to launch muliple rosnodes (e.g. python files) from one terminal command! You also don't need to have `roscore` running! `roslaunch` 

1. Create a launch folder to place your roslaunch files in. Then make your launch script file and make it executable:
```
mkdir src/turtlebot_pubsub/launch
touch turtlebot_pubsub/launch/pubsub.launch
chmod +x turtlebot_pubsub/launch/pubsub.launch
```
2. Make your launch file run both nodes. You can check my launch script at `turtlebot_pubsub/launch/pubsub.launch`.
3. Now launch your launch script in one terminal! You don't need roscore 
4. Everything is going at once!

*Note: you can also launch another launch file within a launch file by adding the `include` argument in your launch file that follows the structure: `<include file="$(find <ros_pkg>)/launch/<roslaunch_file>.launch" />`. For example `<include file="$(find imu)/launch/prebuilt_imu.launch" />`*

## Other stuff that is useful

- **rosparams**: Essentially they're global variables in ROS. You can add them to your roslaunch file as well as access them within you individual rosnode scripts.
- **rosservice**: This is a 2 way communication protocol instead of the publisher/subscriber method (see https://wiki.ros.org/rospy/Overview/Services). For example: Change the color of the line the turtle is drawing with: `rosservice call /turtle1/set_pen 255 0 0 3 0`. Or you can spawn another turtle named chad: `rosservice call /spawn 3 3 0 chad`. Useful commands 
  - `rosservice list`: Just like `rostopic list` but lists services
  - `rosservice info <service_name>`: Just like rostopic and rosnode info.
  - `rosservice call <service_name> <arguments>`: Like `rostopic pub`. You can find out the arguments with `rosservice info <service_name>`.
  - `rossrv show <service_data_type>`: Like `rosmsg info`, but there are 3 dashes that divide it. The variables above the dashes are the service call (i.e. what you send to the rosservice). The variables below the dashes are what is sent back (i.e. the response from the rosservice).