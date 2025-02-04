This is a ROS package to meet the requirements of a recruitment technical challenge.

Dependencies include geometry_msgs and turtlesim. To try out a demo, build this package with catkin, as you would any other ROS package. Don't forget to:
```
source devel/setup.bash
```
or equivalent.
Then launch with:
```
roslaunch jbell-tech-challenge turtlesim_demo.launch
```

The initial condition is a waypoint of x=0, y=0, theta=0. Waypoints may be sent using rostopic pub. An example:
```
rostopic pub -1 /waypoint turtlesim/Pose -- 2.0 3.0 1.0 0.0 0.0
```

The keyboard control uses turtlesim's turtle_teleop_key. Note that the terminal window must be in focus in order for the key strokes to be detected. Up and down arrows command forward and backwards respectively. Left and right arrows command anticlockwise and clockwise turns. 

Known issue: the poses given by the turtlesim warnings are not the same as the published turtle poses. The coordinate system used here is the published turtle pose, which places (0,0) at the bottom left corner, with a vertical y-axis and a horizontal x-axis.

You can run unit tests using:
```
catkin_make run_tests
```
