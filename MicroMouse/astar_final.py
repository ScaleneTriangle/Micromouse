# -*- coding: utf-8 -*-
"""
Created on Fri Dec 13 07:14:02 2019

@author: ajkra
Source of A* search code
https://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/

Modifications have been made to make this work with my type of maze
"""

from maze import Maze
import heapq



testmaze = Maze( r"C:\Users\ajkra\Documents\Courses\DataScientist\Capstone Project\AI_startercode\test_maze_01.txt")

# Need to wrewrite for my maze type
class Cell(object):
    def __init__(self, x, y, cell_walls):
        """
        Initialize new cell

        @param x cell x coordinate
        @param y cell y coordinate
        @param reachable is cell reachable? not a wall in the way?
        """
        # Need to make reachable be based on the walls of the current cell
        self.cell_walls = cell_walls
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0
        # This is what the parent uses to move to this cell
        self.from_parent = None
        
    def is_permissible(self, direction):
        """ 
        Check if the known maze will allow a movement. Does not 
        use sensor inputs.
        
        @param direction a choice of 'u', 'r', 'l', or 'd'
        """
        # Flipping the binary in the current_maze to find if a wall is 
        #   blocking the path
        # Getting the value of the current location
        val = self.cell_walls
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

class AStar(object):
    def __init__(self, maze_dim, current_maze, current_location, goal):
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()
        self.cells = []
        self.grid_height = maze_dim
        self.grid_width = maze_dim
        self.current_maze = current_maze
        self.current_location = current_location
        self.goal = goal
          
    def init_grid(self):
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                cell_walls = self.current_maze[y,x]
                self.cells.append(Cell(x, y, cell_walls))
        self.start = self.get_cell(self.current_location[1], self.current_location[0])
        self.end = self.get_cell(self.goal[1], self.goal[0])
        
    def get_heuristic(self, cell):
        """
        Compute the heuristic value H for a cell: distance between
        this cell and the ending cell multiply by 10.
    
        @param cell
        @returns heuristic value H
        """
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))
    
    def get_cell(self, x, y):
        """
        Returns a cell from the cells list
        @param x cell x coordinate
        @param y cell y coordinate
        @returns cell
        """
        return self.cells[x * self.grid_height + y]
    
    def get_adjacent_cells(self, cell):
        """
        Returns adjacent cells to a cell. Clockwise starting
        from the one on the right.
     
        @param cell get adjacent cells for this cell
        @returns adjacent cells list 
        """
        cells = []
        directions = []
        if cell.x < self.grid_width-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
            directions.append('u')
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
            directions.append('l')
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
            directions.append('d')
        if cell.y < self.grid_height-1:
            cells.append(self.get_cell(cell.x, cell.y+1))
            directions.append('r')
        return cells, directions

    def display_path(self, show=True):
        cell = self.end
        moves = [cell.from_parent]
        while cell.parent is not self.start:
            cell = cell.parent
            moves.append(cell.from_parent)
            if show:
                print('path: cell: %d,%d' % (cell.x, cell.y), cell.from_parent)
        return moves[::-1]
            
    def update_cell(self, adj, cell, move):
        """
        Update adjacent cell
     
        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        adj.g = cell.g + 15
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g
        adj.from_parent = move
        
        
    def process(self):
        # add starting cell to open heap queue
        i=0
        heapq.heappush(self.opened, (self.start.f, i, self.start))
        while len(self.opened):
            # pop cell from heap queue 
            f, _, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, display found path
            if cell is self.end:
                #self.display_path()
                break
            # get adjacent cells for cell
            adj_cells, adj_cells_order = self.get_adjacent_cells(cell)
            #print('main_cell', (cell.x, cell.y))
            #print('adj_cells', [((i.x, i.y),j) for i, j in zip((adj_cells), adj_cells_order)])
            for adj_cell, order in zip(adj_cells, adj_cells_order):
                #print(cell.cell_walls, order, (adj_cell.x, adj_cell.y))
                #print(cell.is_permissible(order))
                if cell.is_permissible(order) and adj_cell not in self.closed:
                    #input()
                    #print('If Permissible: ', adj_cell not in self.closed)
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found for this adj
                        # cell.
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell, order)
                    else:
                        i += 1
                        self.update_cell(adj_cell, cell, order)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, i, adj_cell))
