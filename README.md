# Micromouse Project

## Contents:
- tester.py: This is the function that when run will use robot_final.py to
navigate the maze created by maze.py. Just run tester.py and it will by default
solve test_maze_02.txt located in Test_Mazes. Change filename on line 15 to
select a different maze to solve. Change slow_down on line 13 to True if you
want to slow down the solving of the maze enough to see the robot move through
the maze.
- robot_final.py: This is the function that determine the moves to make given
sensor inputs and the size of the maze. tester.py will pass sensor inputs to
this function.
- astar_final.py: Contains an A* search algorithm that is used by robot_final.py
to explore and solve the maze.
- maze.py: Contains code to convert the maze .txt files into an object that can
be used by tester.py
- showmaze.py: Contains code to create a visualization of the maze .txt files.
Change the filename on line 13 to select a different maze to visualize.
- Test_Mazes: Contains the .txt files for several mazes.
- Maze_Images: Contains the visualization of the .txt files and visualizations
for the README.md
- Archive: Contains previous iterations of code and code that will potentially
be improved.

## Project Overview
This project is inspired by the Micromouse competitions, wherein  a robot
mouse is tasked with plotting a path from a corner of the maze to its center.
The Micromouse will make to runs in a given maze. In the first run the robot
will explore the maze and find the goal at the center. In the second run the
robot will navigate from the start to the center in as few time steps as
possible. A simplified simulated maze environment is provided with the goal of
finding the fastest time possible.

## Problem Statement
As stated in the project overview, the goal is to solve the maze as quickly
as possible. The strategy is to initially explore the maze just enough to find
the fastest possible route from the starting corner to the center of the maze.
Then identify the fastest route from the start to the center and move the robot
through that identified route. The final solution implemented in this code is to
use an A* search algorithm for both initial exploration and final route
determination. Full implementation will be discussed in the Implementation
section.

## Metrics
The metrics to measure the performance are based on the number of time steps
required to explore the maze in the first round and navigate the maze in the
second round. The goal is to minimize the sum of time steps in the first and
second round, but the first round time steps only count 1/30th of a time step
towards the final scoring. Therefore spending time exploring on the first round
is punished much less than spending additional time on the second round. Both
rounds must be completed within 1000 time steps or the maze is considered
"unsolved".

## Data Exploration
The maze exists in an n x n grid of squares, n even. Maze size n may be 12, 14
or 16. The maze below shows the standard formation of the maze.

![Test Maze 1](/Maze_Images/Maze_combined.png?raw=true "Test Maze 1")

The starting square is always at location (0,0) and the goal is always at the
center of the maze in a 2 x 2 square. Each cell of the maze is represented by a
4 bit number where the 0 is a walled edge and a 1 is an open edge. The 1s
register corresponds with the upwards-facing side, the 2s register the right
side, the 4s register the bottom side, and the 8s register the left side.

![Cell Example](/Maze_Images/Cell_example.png?raw=true "Cell Example")

The robot interacts with the maze through obstacle sensors that detect the
number of open squares in the direction of the sensor. There are left, front,
and right facing sensors. Moves passed from the robot are used by the simulation
to update the robot's position and return new sensor values.

## Data Preprocessing
No preprocessing needed. The robot is always considered to be in the center of
the current cell and all sensors and movements are exact.

## Implementation
![Maze1 Being Solved](/Maze_Images/Test_Maze_1_run.gif?raw=true "Maze1 Solve")

The core goal of the robot controller in robot_final.py is to repeatedly use the
A* search algorithm to find the best route to the goal, and then update that
route at each time step as more walls are found. One note on the this version of
the robot is that it will only perform one move at a time, and will not reverse
it's direction. Multiple movements impacted maze exploration because the sensors
do not update while moving between squares. Reversing is on the TODO list.

    # Update the maze walls with the sensor inputs
    self.update_maze(sensors)
    ## Find optimal route to goal
    rotation, movement = self.AStar_robot()
    ## Update Robot internal state
    self.update_robot_state(sensors, rotation, movement)

The path finding of the robot is done in astar_final.py. I used [Laurent Luce's Blog](https://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/)
for how to perform the A* search. I made modifications to the code to make it
work with my type of maze, but full credit on the A* search implementation goes
to Laurent Luce. The A* search works by using a heuristic distance from the goal
(straight line in this case) and the steps from the starting point to
determine which route is optimal. For a much better description visit the linked
blog.

## Refinement
I tried several different types of robot controllers while modifying and
updating how I handled the robot's internal state of the environment.
These robot controllers included:
- Moving in a straight line: No sensors inputs, just trying to return values to
the simulation to move in a straight line. Could not solve the maze.
- Random search: No sensor inputs. Purely random movements. Trying to see how a
random robot would do when exploring the environment. Would sometimes solve the
maze but more often than not Did Not Finish (DNF)
- Moving towards less visited adjacent cells: Using sensor inputs. Building an
internal understanding of the maze and moving towards the cells that have been
visited less. Led to a lot of back tracking, but was able to solve the maze.
- Moving towards less visited adjacent cells with a bias towards cells closer
the goal: Using sensor inputs. Building the internal understanding of the maze
like the previous robot design, but using a heuristic distance from the center
of the maze to choose the path more likely to reach the goal.
- A* Search Algorithm: As described in Implementation.

## Model Evaluation and Validation
![Robot Scores](/Maze_Images/Scores.PNG?raw=true "Robot Scores")

The robustness is shown by successful completion of the three test mazes and the
self generated maze. The one issue I've seen with using the A* search algorithm
for the exploration run is that it will not explore more if it has a *single*
move optimal route.

![Weakness](/Maze_Images/Weakness_of_Astar_Exploration.PNG?raw=true "A* Search Weakness")

To clarify, it will not explore beyond the most direct path
even if moving farther in straight lines would be faster. This is not an issue
for my robots that only perform a single time step for each, but is a weakness
in any future robot designs, simulated or physical.

## Justification
There may be more effective ways to explore the maze, but the A* search
algorithm will always perform optimal navigation and I believe it does a very
good job exploring the maze as well.
As can be seen, the A* algorithm will always try to find the optimal route to
the goal. On the return trip the A* algorithm will always tend towards
unexplored cells because it will believe there is an open path and will map out
any alternative route from the initial route.

## Reflection
The A* search algorithm appears to be an optimal solution for navigating the
maze during the second round and at least a very effective way to explore the
maze during the first round. I found one interesting case where the A* algorithm
got stuck moving back and forth between to identically valued cells, and would
never get out. This was solved by modifying the value of distance from the start
cell in the A* algorithm, but was interesting nonetheless that such a state
existed. Surprisingly, most difficult part was not the search algorithms
themselves, but modifying the search algorithms to properly return movements
that would update the robot's internal estimation of it's position and the maze
to match the state of the simulated environment.

## Improvement
There are quite a few improvements I would make to the design of the simulated
environment:
1) Add up to 3 moves per time step where there are straight lines to travel.
Make sure that straight paths are favored over twisting paths when choosing
the optimal path, to represent how moving fast in a straight line is more useful
than slowly moving through twisting paths.
2) Implement more types of exploration algorithms to determine if the A* search
algorithm is actually the most effective when exploring.
3) Implement in hardware using a Arduino/Raspberry Pi and three ultrasonic range
sensors. There would be quite a few changes needed to make to the code to deal
with a continuous environment rather than the discrete simulated environment,
but it would be very interesting.

## Acknowledgements
- Big thanks to Laurent Luce for the A* Search Tutorial and Code
[Laurent Luce's Blog](https://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/)
- Thanks to the starter code for the simulated environment and framework
- Thanks to all the stackoverflow posts I searched to handle weird pieces of
syntax
