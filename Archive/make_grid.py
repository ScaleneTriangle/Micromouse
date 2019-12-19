# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 06:51:14 2019

@author: ajkra

Make a grid

"""
import numpy as np

n = 16
mygrid = np.zeros((n,n))
for i in range(n):
    for j in range(n):
        mygrid[i,j] = i + j
        
print(mygrid)
filename = r"C:\Users\ajkra\Documents\Courses\DataScientist\Capstone Project\AI_startercode\test_maze_01.txt"
dim = int(np.loadtxt(filename, max_rows=1))
walls = np.loadtxt(filename, delimiter=',',skiprows=1, dtype=int)
#walls[-1,-1] = 0
wall_errors = []
# vertical walls
for x in range(dim-1):
    for y in range(dim):
        if (walls[x,y] & 2 != 0) != (walls[x+1,y] & 8 != 0):
            wall_errors.append([(x,y), 'v'])
# horizontal walls
for y in range(dim-1):
    for x in range(dim):
        if (walls[x,y] & 1 != 0) != (walls[x,y+1] & 4 != 0):
            wall_errors.append([(x,y), 'h'])