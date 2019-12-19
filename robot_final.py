import numpy as np
from astar_final import AStar

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
        self.goal_bounds = [self.maze_dim//2 - 1, self.maze_dim//2]
        self.goal = (self.goal_bounds[0], self.goal_bounds[0])
        self.current_sensors = (0,0,0)
        self.left_cell = []
        self.front_cell = []
        self.right_cell = []
        self.goal_found = False
        
        # Maze state, starting off assuming no walls
        self.current_maze = np.ones((maze_dim, maze_dim), dtype=int) * 15
        self.move_list = []
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
        # Set map_return_path to true if want robot to try finding a faster
        #   path from the goal
        map_return_path = True
        rotation = 0
        movement = 0
        # Update the maze based on sensor values
        self.update_maze(sensors)
        
        ## Find optimal route to goal
        print(self.location, self.heading, self.goal_bounds[0])
        rotation, movement = self.AStar_robot()
        ## Update Robot internal state
        self.update_robot_state(sensors, rotation, movement) 
        
        ## Check if the first goal has been reached, return to (0,0)
        if map_return_path:
            if (self.goal_found == False and 
                self.location[0] in self.goal_bounds and 
                self.location[1] in self.goal_bounds):
                # If goal is reached, set goal to return back to (0,0)
               self.goal =  (0,0)
               self.goal_found = True
            
            if (self.goal_found == True and self.location == [0,0]):
                self.location = [0,0]
                self.heading = 'u'
                self.goal = (self.goal_bounds[0], self.goal_bounds[0])
                return('Reset','Reset')
        else:
            if (self.location[0] in self.goal_bounds and 
                self.location[1] in self.goal_bounds):
                # Need to reset one more time to let tester catch up
                if self.goal_found == True:
                    self.location = [0,0]
                    self.heading = 'u'
                    self.goal = (self.goal_bounds[0], self.goal_bounds[0])
                    self.goal_found = False
                    return('Reset','Reset')
                self.goal_found = True
            
        # Return the move the robot will make
        return rotation, movement
        
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
        
        ## Use bitwise operations to add a wall based on the representation
        # Note 14 (1110) == top wall; 13 (1101) == right wall;
        #      11 (1011) == bottom wall; 7 (0111) == left wall
        # Performing the & operation leaves the previous walls and adds a new
        #    wall where needed
        if side == 'u' or side == 'up':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 14
        if side == 'r' or side == 'right':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 13
        if side == 'd' or side == 'down':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 11
        if side == 'l' or side == 'left':
            self.current_maze[cel_pos[0], cel_pos[1]] = self.current_maze[cel_pos[0], cel_pos[1]] & 7
            
    def AStar_robot(self):
        """
        Perform a AStar search to find the optimal route based on the
        known maze 
        
        Note: Changing the weight of g (weight based on steps from start)
            in AStar caused the algorithm to get out of a stuck corner case.
            Should do something to modify cell.g if the same move is seen 
            multiple times in a short period. (3 in 5?)
        """
        search = AStar(self.maze_dim, self.current_maze, 
                       self.location, self.goal)
        # Build the internal AStar grid
        search.init_grid()
        # Find the optimal route
        search.process()
        # Return the list of moves necessary to reach the goal
        self.move_list = search.display_path(show=False)
        print(self.move_list)
        #print(self.move_list)
        # Return the move
        rotation, movement = self.move_direction(self.move_list.pop(0))
        return rotation, movement
    
    def is_permissible(self, sensors, rotation, movement):
        """
        Use the sensors to check if trying to run into a wall
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
        Update the robot's internal estimated location and 
        heading based on a planned rotation and movement
        
        Mostly copied from the tester.py code
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
    
    def move_direction(self, direction):
        """ 
        Return the rotation and movement to go a direction
        Input: direction(str): 'u', 'r', 'd', or 'l' 
        Output: rotation(int): -90, 0, or 90 depending on turn needed
                movement(int): 0 or 1 depending on number of moves 
        """
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
        """ 
        Prints the maze. Puts a -1 at the robot's current location
        Input: which_maze(str): 'layout' for the walls
                                'visited' for the cells visited
                                'heuristic' for the distances from the center
        """
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
    

        
        
if __name__ == "__main__":
    test = Robot(12)
    test.next_move([0,10,0])
    test.show_maze()
    