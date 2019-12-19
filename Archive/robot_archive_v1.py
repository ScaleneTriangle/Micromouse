import numpy as np
import random as rand
import time
from A_star_search_my_maze import AStar

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        # Useful dictionaries from tester.py
        self.dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                       'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                       'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                       'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
        self.dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
                    'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
        self.dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
                       'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}


        self.location = [0, 0]
        self.heading = 'u'
        self.maze_dim = maze_dim
        self.goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.current_sensors = (0,0,0)
        self.left_cell = []
        self.front_cell = []
        self.right_cell = []
        self.goal_found = False
        self.run_2_flag = False
        
        # Maze state, starting off assuming no walls
        self.current_maze = np.ones((maze_dim, maze_dim), dtype=int) * 15
        self.locations_visited = np.zeros((maze_dim, maze_dim), dtype=int)
        
        self.heuristic_maze = np.zeros((maze_dim, maze_dim), dtype=int)
        self.build_heuristic_maze()
        
        self.past_locations = []
        self.past_headings = []
        # Robot control needs

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        # Log Current Location (Class or function)
        rotation = 0
        movement = 0
        self.update_visited_locations()
        # Sensors is a list ordered as [L, F, R], each can be [0 to max]
        
        # Assess/update state of maze, do something to determine legal moves
        self.update_maze(sensors)
        # Pass legal moves to robot controller (Class or Function)
        # Random Robot that will only perform legal moves
        print("Sensors: ", sensors)
        
        
        ## TODO: NEED TO SETUP ASTAR SEARCH make sure it works first
        if not self.run_2_flag:
            n = 0
            while True:
                #rotation, movement = self.random_robot(sensors)
                #rotation, movement = self.move_straight_robot(sensors)
                #rotation, movement = self.visit_new_robot(sensors)
                rotation, movement = self.heuristic_robot(sensors)
                if self.is_permissible(sensors, rotation, movement):
                    break    
                n += 1
                if n > 2:
                    raise Exception("Shit's broke")
        else:
            search = AStar(self.maze_dim, self.current_maze)
        # do A*search here when maze is explored            

        print("Robot Action: ", rotation, movement) 
        
        
        
        
        
        # Log Move Made and any other considerations being tracked (Same Logger)
        
        # Output should be (rotation = R[-90, 0, 90], movement = R[0,1,2,3] 
        # Output One tuple per turn
        # Output ('Reset', 'Reset') ends the run
        
        if self.goal_found == True:
            self.location = [0,0]
            self.heading = 'u'
            self.run_2_flag = True
            print('gottem')
            self.goal_found = False
            # REMOVE THE FOLLOWING LINE ONCE DOING THE REAL STUFF
            #self.current_maze = np.ones((self.maze_dim, self.maze_dim), dtype=int) * 15
            return('Reset','Reset')
        
        self.update_robot_state(sensors, rotation, movement)
        if self.location[0] in self.goal_bounds and self.location[1] in self.goal_bounds:
            self.goal_found = True
            
        
        return rotation, movement
    
    
    def logger(self, ):
        self.locations = self.location
        return None
    def update_visited_locations(self):
        self.locations_visited[self.location[0], self.location[1]] += 1
        
    def update_maze(self, sensors):
        """ Used to update the internal maze based on robot sensor inputs """
        # NOTE: Rotations are backwards (ccw is pos) for this rotation matrix
        rotations = {'u': 0*np.pi/180, 'l': 90*np.pi/180,
                     'r': -90*np.pi/180, 'd': 180*np.pi/180}
        head = self.heading
        
        # Put walls where the sensors see them
        # Wall located in front
        left_cell_to_wall = np.array([-sensors[0], 0])
        front_cell_to_wall = np.array([0, sensors[1]])
        right_cell_to_wall = np.array([sensors[2], 0])
        #Rotate the cells_to_wall
        rotation_matrix = np.rint(np.array([[np.cos(rotations[head]), -np.sin(rotations[head])],
                                    [np.sin(rotations[head]), np.cos(rotations[head])]]))
        left_cell = np.matmul(rotation_matrix, left_cell_to_wall) + np.array([self.location])[0]
        front_cell = np.matmul(rotation_matrix, front_cell_to_wall) + np.array([self.location])[0]
        right_cell = np.matmul(rotation_matrix, right_cell_to_wall) + np.array([self.location])[0]
        
        # Add the walls to the cells
        # Rotating the directions because I thought it would be slick
        left_wall_side = str(list(np.matmul(rotation_matrix, np.array([-1, 0]))))
        front_wall_side = str(list(np.matmul(rotation_matrix, np.array([0, 1]))))
        right_wall_side = str(list(np.matmul(rotation_matrix, np.array([1, 0]))))
        
        # Actually adding the walls
        dir_dict = {'[0.0, 1.0]': 'u', '[0.0, -1.0]': 'd', '[1.0, 0.0]': 'r', '[-1.0, 0.0]': 'l'}
        self.add_wall(left_cell, dir_dict[left_wall_side])
        self.add_wall(front_cell, dir_dict[front_wall_side])
        self.add_wall(right_cell, dir_dict[right_wall_side])

        # FOR USE IN OTHER FUNCTIONS, normalizing the cell directions
        self.left_cell = np.matmul(rotation_matrix, np.array([-1, 0])) + np.array([self.location])
        self.front_cell = np.matmul(rotation_matrix, np.array([0, 1])) + np.array([self.location])
        self.right_cell = np.matmul(rotation_matrix, np.array([1, 0])) + np.array([self.location])
    
    def add_wall(self, cel_pos, side):
        """ Used to add a wall to the internal maze """
        cel_pos = list(cel_pos.astype(np.int64))

        if side == 'u' or side == 'up':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 14
        if side == 'r' or side == 'right':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 13
        if side == 'd' or side == 'down':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 11
        if side == 'l' or side == 'left':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 7
            
    def random_robot(self, sensors):
        """ Random actions """ 
        who_cares = sensors
        rotation = rand.choice([-90, 0, 90])
        movement = rand.choice([0, 1])  #, 2, 3]) removing more moves for now
        return rotation, movement
    
    def move_straight_robot(self,sensors):
        """ Go in a straight line"""
        who_cares = sensors
        rotation = 0
        movement = 1
        return rotation, movement
   
    def visit_new_robot(self, sensors):
        """ Prefer going to unvisited areas """
        # Check current position
        cur_pos = self.location
        # Check adjacent areas
        ## Dictionary for adjacent areas NOTE: self.dir_move is the same... but as lists
        adj_dict = {'u':np.array([0,1]),
                    'r':np.array([1,0]),
                    'd':np.array([0,-1]), 
                    'l':np.array([-1,0])} # Remove probably
        # Move to least visited adjacent area
        ## Find least visited adjacent
        adj_cells = np.vstack((self.left_cell, self.front_cell, self.right_cell))
        #print(adj_cells, adj_cells[0])
        # How many times the adjacent cells have been visited
        adj_vis = np.array([0,0,0])
        for i, loc in enumerate(adj_cells):
            if loc[0] < 0 or loc[0] >= self.maze_dim:
                adj_vis[i] = 1000
            elif loc[1] < 0 or loc[1] >= self.maze_dim:
                adj_vis[i] = 1000
            else:
                adj_vis[i] = self.locations_visited[int(loc[0]), int(loc[1])]

        # Which cell has been visited the least    
        min_cell = adj_vis == adj_vis.min()
        
        # Setup for the 3 directions of left front right
        directions = ['l', 'f', 'r']
        # Create an array of which are the minimums    ex/['', 'f', 'r']
        desired_directions = [min_cell[i] * directions[i] for i in range(3)]
        print("Desired_Directions: " , desired_directions)
        # NOTE: need to make a function that will return a movement and rotation
        #    based on current location and desired location (relative or absolute)
        for direct in desired_directions:
            print("Direct: ", direct)
            if direct == 'l':
                rotation = -90
                movement = 1
                if self.is_permissible(sensors, rotation, movement):
                    break
                else:
                    for rot, mov in [(90, 1), (-90, 1), (0,1), (90,0), (-90,0)]:
                        rotation = rot
                        movement = mov
                        if self.is_permissible(sensors, rotation, movement):
                            print('Here: ', sensors, rotation, movement)
                            break
            elif direct == 'f':
                rotation = 0
                movement = 1
                if self.is_permissible(sensors, rotation, movement):
                    break
                else:
                    for rot, mov in [(90, 1), (-90, 1), (0,1), (90,0), (-90,0)]:
                        rotation = rot
                        movement = mov
                        if self.is_permissible(sensors, rotation, movement):
                            print('Here: ', sensors, rotation, movement)
                            break
            elif direct == 'r':
                rotation = 90
                movement = 1
                if self.is_permissible(sensors, rotation, movement):
                    break
                else:
                    for rot, mov in [(90, 1), (-90, 1), (0,1), (90,0), (-90,0)]:
                        rotation = rot
                        movement = mov
                        if self.is_permissible(sensors, rotation, movement):
                            print('Here: ', sensors, rotation, movement)
                            break

                        
        print('Internal rotation/movement: ', rotation, movement)    
        return rotation, movement
        
    def heuristic_robot(self, sensors):
        loc = self.location
        # Pull the adjacent cell's heuristics and num times visited
        #   There has to be a better way to do this...
        
        # NEED TO HAVE A BETTER WAY TO DO THE HEURISTIC MAZE HANDLING
        
        adj_cells = [np.array(loc)+np.array(self.dir_move['u']),
                     np.array(loc)+np.array(self.dir_move['r']),
                     np.array(loc)+np.array(self.dir_move['d']),
                     np.array(loc)+np.array(self.dir_move['l'])]
        
        #adj_vis = np.array([0,0,0,0])
        #adj_heur = np.array([0,0,0,0])
        adj_cell_cost = np.array([0,0,0,0])
        for i, loc in enumerate(adj_cells):
            if loc[0] < 0 or loc[0] >= self.maze_dim:
                adj_cell_cost[i] = 2000
            elif loc[1] < 0 or loc[1] >= self.maze_dim:
                adj_cell_cost[i] = 2000
            else:
                adj_cell_cost[i] = ( 
                          self.heuristic_maze[int(loc[0]), int(loc[1])]    
                        + self.locations_visited[int(loc[0]), int(loc[1])]*20)

        # THIS ISN'T WORKING, NEED TO DO SOME THINKING

        # Loop through options until find one that works/is best
        directions = ['u', 'r', 'd', 'l']
        while True:
            # Make a boolean of the lowest cost cell to choose
            min_cell = adj_cell_cost == np.min(adj_cell_cost)
            min_cell = min_cell.astype(np.int32)
            # Create an array of which are the minimums    ex/['', 'f', 'r']
            desired_directions = [min_cell[i] * directions[i] for i in range(4)]
            print('desired_directions: ', desired_directions)
            for i, move in enumerate(desired_directions):
                # Return the movements and check if the move is permissible
                if move == '' and adj_cell_cost[i] != 2000:
                    adj_cell_cost[i] += 100
                    continue
                print('In move {}'.format(move))
                print('Adj_cell_cost {}: '.format(i), adj_cell_cost)
                rotation, movement = self.move_direction(move)
                if self.is_permissible(sensors, rotation, movement):
                    # If the move is permissible return it
                    print("In robot location: ", self.location, self.heading)
                    print("Adj Cells: ", adj_cells)
                    print('adj_cell_cost: ', adj_cell_cost)
                    
                    return rotation, movement
                    
                    
                else:
                    # If the move is not permissible there is probably a wall
                    #  change the cell cost to 1000 so it won't be picked again
                    adj_cell_cost[i] = 2000    
        raise Exception('Something got goofed.')
    
    def is_permissible(self, sensors, rotation, movement):
        """
        Just check if trying to run into a wall
        """
        # Check if dead end, if dead end: must rotate
        if sensors == [0,0,0] and rotation != 0 and movement == 0:
            return True
        else:
        # Check if turn is trying to move towards a wall and not in dead end
            if sensors[0] == 0 and rotation == -90:
                return False
            if sensors[1] == 0 and rotation == 0:
                return False
            if sensors[2] == 0 and rotation == 90:
                return False
            if rotation == 0 and movement == 0:
                # Make sure moving
                return False
            return True
    
    def update_robot_state(self, sensors, rotation, movement):
        """ 
        Update the robot's location and heading based on a planned
        rotation and movement
        """
        # Update the heading
        if rotation == -90:
            self.heading = self.dir_sensors[self.heading][0]
            moves_remaining = sensors[0]
        elif rotation == 90:
            self.heading = self.dir_sensors[self.heading][2]
            moves_remaining = sensors[2]
        elif rotation == 0:
            moves_remaining = sensors[1]
            if movement < 0:
                moves_remaining = abs(movement)
        
        if moves_remaining >= movement:
            moves_remaining = abs(movement)
        # Update Position
        while moves_remaining:
            if movement > 0:
                if moves_remaining > 0:
                    self.location[0] += self.dir_move[self.heading][0]
                    self.location[1] += self.dir_move[self.heading][1]
                    moves_remaining -= 1
                else:
                    #print("Movement stopped by wall.")
                    moves_remaining = 0
            if movement < 0:
                if moves_remaining > 0:
                    self.location[0] += self.dir_move[self.dir_reverse[self.heading]][0]
                    self.location[1] += self.dir_move[self.dir_reverse[self.heading]][1]
                    moves_remaining -= 1
                else:
                    #print("Movement stopped by wall.")
                    moves_remaining = 0        
        print("Internal Robot State: ", 'location: ', self.location, 'heading: ', self.heading)
    
    def move_direction(self, direction):
        """ Return the rotaiton and movement to go a direction """
        # Given heading X and direction Y, return necessary rotation 
        #     and movement with rot, move = action_dict[X][Y]
        if direction not in ['u','r','d','l']:
            return 0,0
        action_dict = {'u': {'u':[0,1],'r':[90,1],'d':[90,0],'l':[-90,1]},
                       'r': {'u':[-90,1],'r':[0,1],'d':[90,1],'l':[90,0]},
                       'd': {'u':[90,0],'r':[-90,1],'d':[0,1],'l':[90,1]},
                       'l': {'u':[90,1],'r':[90,0],'d':[-90,1],'l':[0,1]}}
        rotation, movement = action_dict[self.heading][direction]
        
        return rotation, movement
    
    def show_maze(self, which_maze='layout'):
        if which_maze == 'layout': 
            maze = self.current_maze.copy()
            maze_show = maze.copy()
            maze_show[self.location[0], self.location[1]] = -1
            print(maze_show.T[::-1,:])
        if which_maze == 'visited':
            maze = self.locations_visited.copy()
            maze_show = maze.copy()
            maze_show[self.location[0], self.location[1]] = -1
            print(maze_show.T[::-1,:])
        if which_maze == 'heuristic':
            maze = self.heuristic_maze.copy()
            maze_show = maze.copy()
            maze_show[self.location[0], self.location[1]] = -1
            print(maze_show.T[::-1,:])
        return np.rint(maze.astype(np.int32))
    
    def build_heuristic_maze(self):
        """ Build a grid based on cell distance from the goal """
        for x in range(len(self.current_maze[0])):
            for y in range(len(self.current_maze[1])):
                self.heuristic_maze[x,y] = 10 * (
                    abs(self.goal_bounds[0] - x)
                  + abs(self.goal_bounds[0] - y))
    
    def is_permissible_no_sensor(self, direction):
        """ 
        Check if the known maze will allow a movement. Does not 
        use sensor inputs.
        """
        # Flipping the binary in the current_maze to find if a wall is 
        #   blocking the path
        # Getting the value of the current location
        val = self.current_maze[tuple(self.location)]
        # Finding the not of the binary ex/(from 1110 finding 0001)
        not_val = ((1 << 4) - 1 - val)
        # Use bitwise and to see if there is a wall ex/(1101 & 0001 == 0001 which means wall on top)
        if direction == 'u':
            if (not_val & 1 == 1):
                return False
        if direction == 'r':
            if (not_val & 2 == 2):
                return False
        if direction == 'd':
            if (not_val & 4 == 4):
                return False
        if direction == 'l':
            if (not_val & 8 == 8):
                return False
        # If no walls are seen return that the move is permissible
        return True
    
    def choose_direction():
        pass
        
        
        
if __name__ == "__main__":
    test = Robot(12)
    
    test.next_move([0,10,0])
    test.show_maze()
    