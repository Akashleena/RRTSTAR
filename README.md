# RRT* move_base plugin

The quadcopter_navigation package can be found [here](https://github.com/Akashleena/quadSimulator)


In Terminal 1
```
#Runs the Gazebo world

roslaunch quadcopter_gazebo quadcopter.launch
```

In Terminal 2

```
# Runs the code for hovering the drone

roslaunch quadcopter_takeoff_land quadcopter_takeoff_land.launch

```

In Terminal 3

```
Runs the rviz visualization tool and allows 2D nav goal for navigation in Rviz

roslaunch quadcopter_navigation quadcopter_move_base.launch

```

In Terminal 4

```
Lands the drone

rostopic pub /quadcopter_land -r 5 std_msgs/Empty "{}"

```

