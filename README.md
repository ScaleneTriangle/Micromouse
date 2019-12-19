https://guides.github.com/features/mastering-markdown/
TODO: Images,
Embed Images, Code,
  Implementation section, Refinement section,
  Evaluation Section, make a table of the scores of each robot
![Alt text](/relative/path/to/img.jpg?raw=true "Optional Title")

# Micromouse code.

## Contents:
- tester.py: This is the function that when run will use robot_final.py to
navigate the maze created by maze.py. Just run tester.py and it will by default
solve test_maze_02.txt located in Test_Mazes. Change filename on line 15 to
select a different maze to solve.
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
- Maze_Images: Contains the visualization of the .txt files
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
![Test Maze 1](/Maze_Images/Maze_1.PNG?raw=true "Test Maze 1")
The starting square is always at location (0,0) and the goal is always at the
center of the maze in a 2 x 2 square. Each cell of the maze is represented by a
4 bit number where the 0 is a walled edge and a 1 is an open edge. The 1s
register corresponds with the upwards-facing side, the 2s register the right
side, the 4s register the bottom side, and the 8s register the left side.
[Embed image of 4 bit with lines to edge of wall]

The robot interacts with the maze through obstacle sensors that detect the
number of open squares in the direction of the sensor. There are left, front,
and right facing sensors. Moves passed from the robot are used by the simulation
to update the robot's position and return new sensor values.
- How does the robot interpret the environment

## Data Visualization
- Picture of Maze drawn. Picture of maze in text form. Picture of the moves on
the solved maze. (Do maze 2 to explain the neatness with getting stuck)

## Data Preprocessing
No preprocessing needed. The robot is always considered to be in the center of
the current cell and all sensors and movements are exact.

## Implementation
NOTE DOING ONLY ONE MOVE PER TIME STEP AND NO REVERSE MOVES
Robot Controller loop writeup
[Embed code]
AStar code writeup
[Embed code]

## Refinement
Show the different types of robots I tried
show the random robot, the visited locations robot, and the heuristic robot

## Model Evaluation and Validation
- Bench Mark would be the amount to visit every single cell (assuming no
  backtracking) and then performing a optimal route to the goal.
- Table with Scores of the different models.
The robustness is shown by successful completion of the three test mazes and the
self generated maze.

## Justification
    There may be more effective ways to explore the maze, but the A* search
algorithm will always perform optimal navigation and I believe it does a very
good job exploring the maze as well.
[Embed image of the explore out and explore back to show A* exploration]
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
1a) Make sure that straight paths are favored over twisting paths when choosing
the optimal path, to represent how moving fast in a straight line is more useful
than slowly moving through twisting paths.
2) Implement more types of exploration algorithms to determine if the A* search
algorithm is actually the most effective when exploring.
3) Implement in hardware using a Arduino/Raspberry Pi and three ultrasonic range
sensors. There would be quite a few changes needed to make to the code to deal
with a continuous environment rather than the discrete simulated environment,
but it would be very interesting.

## Acknowledgements
- Shoutouts to the starter code and laurent luce's description of A* maze solve
