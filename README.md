# SelfDrivingTurtle

## Introduction
The turtlebot follows a yellow track and acts differently at intersections based on visual information. This is simulated in a Gazebo simulator.

### Usage
To launch turtlebot and map for [part 1](#Path-1-preparation)
```
roslaunch followbot launch.launch
python part2.py
```

To launch turtlebot and map for [part 2](#Path-2-map-with-color-markers)
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
python part2.py
```

To launch turtlebot and map for [part 3](#Path-3-map-with-shape-markers)
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
python part3.py 
```

### Path 1: Preparation
Follows yellow line with error adjustment to better follow the yellow track. 

![alt text][f1]

### Path 2: Map with color markers
Employs color recognition to 
green: turn left 
blue: turn right 
red: stop 

This is done by masking over a portion of the image once left or right is decided, 
therefore the yellow tracking finds the center of the correct yellow path.
This same method is used in part 3 with a different turn decision.  

![alt text][f2]

### Path 3: Map with shape markers
Employs object matching from opencv library to 
left triangle: turn left 
right triangle: turn right 
star: stop 

Analyzes the frames and votes on what pattern is being recognized since this changes frame to frame. 
Finalized pattern is used to make the turn decision. 

![alt text][f3]

[Base Code from](https://github.com/jingxixu/followbot)


[f1]:https://github.com/s-abdullah/SelfDrivingTurtle/blob/master/gifs/f1.gif
[f2]:https://github.com/s-abdullah/SelfDrivingTurtle/blob/master/gifs/f2.gif
[f3]:https://github.com/s-abdullah/SelfDrivingTurtle/blob/master/gifs/f3.gif