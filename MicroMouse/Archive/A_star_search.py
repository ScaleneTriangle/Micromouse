# -*- coding: utf-8 -*-
"""
Created on Fri Dec 13 07:14:02 2019

@author: ajkra
Source of A* search code
https://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/

modifications have been made to make this work with my type of maze
"""

from maze import Maze
import heapq

testmaze = Maze( r"C:\Users\ajkra\Documents\Courses\DataScientist\Capstone Project\AI_startercode\test_maze_01.txt")

# Need to wrewrite for my maze type
class Cell(object):
    def __init__(self, x, y, reachable):
        """
        Initialize new cell

        @param x cell x coordinate
        @param y cell y coordinate
        @param reachable is cell reachable? not a wall?
        """
        # Need to make reachable be based on the walls of the current cell
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0
        
        
    # Write this function similar/identical to is_permissible_no_sensor    
    def is_reachable(self, current_maze_value):
        return False

class AStar(object):
    def __init__(self):
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = set()
        self.cells = []
        self.grid_height = 6
        self.grid_width = 6
        
    # Need to rewrite for my maze type    
    def init_grid(self):
        walls = ((0, 5), (1, 0), (1, 1), (1, 5), (3, 3), 
                 (3, 1), (3, 2), (3, 5), (4, 1))
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(x, y, reachable))
        self.start = self.get_cell(0, 0)
        self.end = self.get_cell(5, 5)
        
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
        if cell.x < self.grid_width-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < self.grid_height-1:
            cells.append(self.get_cell(cell.x, cell.y+1))
        return cells

    def display_path(self):
        cell = self.end
        while cell.parent is not self.start:
            cell = cell.parent
            print('path: cell: %d,%d' % (cell.x, cell.y))
            
    def update_cell(self, adj, cell):
        """
        Update adjacent cell
     
        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g
        
    def process(self):
        # add starting cell to open heap queue
        i = 0
        heapq.heappush(self.opened, (i, self.start.f, self.start))
        while len(self.opened):
            # pop cell from heap queue 
            _, f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, display found path
            if cell is self.end:
                self.display_path()
                break
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found for this adj
                        # cell.
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        i += 1
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        #heapq.heappush(self.opened, (adj_cell.f, adj_cell))
                        heapq.heappush(self.opened, (i, adj_cell.f, adj_cell))


if __name__ == "__main__":
    search = AStar()
    search.init_grid()
    search.process()