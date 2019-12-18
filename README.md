Micromouse code.

Contents:
X
Y
Z

Project Overview
- Inspired by Micromouse competition. Robot plot's paths from a corner to the
center of the maze.
- Two runs, explore then fast solve
- The project will be done entirely in a simulated environment.

Problem Statement
- Build code to explore and solve a maze as fast as possible. Total time in the
maze during exploration and solving may not exceed 1000 time steps.

Metrics
- Goal is to minimize the sum of the scores during the exploration and solving
phases. During exploration phase score is equal to the number of
time steps * 1/30th. During the solving phase score is equal to the number of
time steps.

Data Exploration
- Maze is based on
- Sensor inputs are
- How does the robot interpret the environment

Data Visualization
- Picture of Maze drawn. Picture of maze in text form. Picture of the moves on
the solved maze. (Do maze 2 to explain the neatness with getting stuck)

Data Preprocessing
- No preprocessing needed. All data is clean and simulated. Elaborate as needed

Implementation
- Put code here with description

Refinement
-Show the different types of robots I tried

Model Evaluation and Validation
- Bench Mark
- Table with Scores of the different models.
- Validate robustness. Make a crazy maze and see if that screws anything up

Justification
- Final Result discussion

Reflection
- Final thoughts summary
- Discuss end-to-end problem
- What was interesting

Improvement
- "Future Research"
-- Add multiple moves where the move_list has multiple in a row
-- Setup something for full maze Exploration
-- setup something to detect a stuck condition and change parameters as needed

Acknowledgements
- Shoutouts to the starter code and laurent luce's description of A* maze solve
