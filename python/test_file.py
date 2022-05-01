#file solely for testing functions
import argparse
from math import sqrt
from pathlib import Path
from typing import Callable, Dict

from instance import Instance
from point import Point
from solution import Solution
from file_wrappers import StdinFileWrapper, StdoutFileWrapper

#from ortools.linear_solver import pywraplp
import solve


testpt1 = Point(1, 2)
testpt2 = Point(2, 2)
testpt3 = Point(10,10)
points = [testpt1,testpt2,testpt3]
inst = Instance(30,3,8,points)
sol = solve.solve_greedy(inst)
dictionary = {0:1,1:1,2:3}
print(sol)