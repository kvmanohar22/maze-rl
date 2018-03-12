# Turtle-Sim

## Problem Statement

You will be given a file containing a matrix of `0’s` and `1’s`. `0’s` represent free path and `1’s` represent obstacles. In a ROS node, first of all move the turtle sim to the top left corner of the window. Then you have to map the matrix for the turtlesim window  and traverse the window avoiding the obstacles, to finally reach the opposite corner of the window.

Example Matrix:

```
0 0 1 0 0 0 0 0 1 1 0 0 0
0 0 1 0 0 0 0 0 1 1 0 0 0
0 0 1 0 0 1 0 0 1 1 0 0 0
0 0 0 0 0 1 0 0 0 0 0 0 0
0 0 0 0 0 1 0 0 0 0 0 0 0
```

## Dependencies

- ROS Kinetic


## Usage

- Starting the simulator
   Run the last command in another terminal
   ```bash
   $ rosmake turtlesim
   $ roscore
   $ rosrun turtlesim turtlesim_node
   ```
