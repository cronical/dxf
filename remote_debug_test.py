"""Place to try breakpoints with no import problems
"""
from itertools import compress
import os
import numpy as np
import bpy

print("Start")
print(os.getcwd())
print(np.array([0,3,8]))
print(list(compress([0,3,8],[0,1,0])))
print(type(bpy))
print("End")