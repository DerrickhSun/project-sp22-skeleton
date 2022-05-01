"""Solves an instance.

Modify this file to implement your own solvers.

For usage, run `python3 solve.py --help`.
"""

import argparse
from math import sqrt
import math
from pathlib import Path
from typing import Callable, Dict

from instance import Instance
from point import Point
from solution import Solution
from file_wrappers import StdinFileWrapper, StdoutFileWrapper

from ortools.linear_solver import pywraplp


def solve_naive(instance: Instance) -> Solution:
    return Solution(
        instance=instance,
        towers=instance.cities,
    )

#takes a single tower and a solution
#returns the cost the single tower "sees" if in the solution
#not intended to be used on towers not in the solution
#note that solution.py has a function for total cost
def tower_cost(point, sol):
    penalties = 0
    for i in sol.towers:
        if point.distance_obj(i)<=sol.instance.penalty_radius and point.distance_obj(i)!=0:
            penalties += 1
    return 170*math.e**(0.17*penalties)

#given a point, generates the set of that point
def generate_set(point: Point, instance: Instance):
    result = []
    for i in instance.cities:
        if point.distance_obj(i) < instance.coverage_radius:
            result.append(i)
    return result

#given dictionary, inverts it
def invert_dict(dictionary):
    try:
        result = {}
        for i in dictionary.items():
            if i[1] not in result.keys():
                result[i[1]] = [i[0]]
            else:
                result[i[1]].append(i[0])
        return result
    except: #handles the case where dict values are nonhashable
        result = {} #uses frozenset - no changing afterwards!
        for i in dictionary.items():
            if frozenset(i[1]) not in result.keys():
                result[frozenset(i[1])] = [i[0]]
            else:
                result[frozenset(i[1])].append(i[0])
        return result



def solve_greedy(instance: Instance) -> Solution:
    points_to_sets = {}
    for i in range(instance.grid_side_length):
        for j in range(instance.grid_side_length):
            pt = Point(i,j)
            pt_set = generate_set(pt,instance)
            if len(pt_set)>0: #ignore points that cover nothing
                points_to_sets[pt] = pt_set
    sets_to_points = invert_dict(points_to_sets)
    sets = [list(i) for i in sets_to_points.keys()]

    chosen_sets = []
    unselected_cities = instance.cities
    selected_cities = []
    def unselected_cities_left(target):
        count = 0
        for i in target:
            if i not in selected_cities:
                count+=1
        return count
    while len(unselected_cities)>0:
        #helper function
        #takes in a set of cities and set of already covered cities
        #returns number of cities in the set still uncovered
        sets.sort(key = unselected_cities_left, reverse = True)
        selected = sets[0]
        chosen_sets.append(selected)
        for i in selected:
            if i in unselected_cities:
                unselected_cities.remove(i)
                selected_cities.append(i)
    
    chosen_towers = []
    for i in chosen_sets:
        chosen_towers.append(sets_to_points[frozenset(i)][0])
    
    return Solution(chosen_towers, instance)

def create_data_model(instance: Instance):
    data = {}
    d2 = instance.D**2 #returns side length squared = size of grid
    data["num_vars"] = d2
    data["num_constraints"] = instance.N #returns number of cities
    data["bounds"] = [1] * d2
    data["constraint_coeffs"] = [[0] * d2 for _ in range(instance.N)]

    for k, point in enumerate(instance.cities):
        x, y = point.x, point.y
        constraint = data["constraint_coeffs"][k]
        for i in range(instance.D):
            for j in range(instance.D):
                if sqrt((x - i) ** 2 + (y - j) ** 2) <= instance.R_s:
                    constraint[i * instance.D + j] = 1

    data["obj_coeffs"] = [1] * d2
    return data



#solves set cover via LP
def solve_cover(instance: Instance) -> Solution:
    data = create_data_model(instance) #creates a dict with instance information
    solver = pywraplp.Solver.CreateSolver("SCIP")

    #creates inequalities for LP
    #solver is from pywraplp, which is a tool to solve lp problems
    x = {}
    for j in range(data["num_vars"]): #data[num_vars] is the size of the grid
        x[j] = solver.IntVar(0, 1, "x[%i]" % j)

    infinity = solver.infinity()
    for i in range(data["num_constraints"]):
        constraint = solver.RowConstraint(-infinity, -data["bounds"][i], "")
        for j in range(data["num_vars"]):
            constraint.SetCoefficient(x[j], -data["constraint_coeffs"][i][j])

    objective = solver.Objective()
    for j in range(data["num_vars"]):
        objective.SetCoefficient(x[j], data["obj_coeffs"][j])
    objective.SetMinimization()

    status = solver.Solve()
    if status != pywraplp.Solver.OPTIMAL:
        print(f"Solver not optimal: {status}", file=sys.stderr)

    #rounds the results of LP
    towers = []
    for k in range(data["num_vars"]):
        if int(x[k].solution_value()) == 1:
            i = k // instance.D
            j = k % instance.D
            towers.append(Point(i, j))

    return Solution(instance=instance, towers=towers)


SOLVERS: Dict[str, Callable[[Instance], Solution]] = {
    "naive": solve_naive,
    "cover": solve_cover,
}


# You shouldn't need to modify anything below this line.
def infile(args):
    if args.input == "-":
        return StdinFileWrapper()

    return Path(args.input).open("r")


def outfile(args):
    if args.output == "-":
        return StdoutFileWrapper()

    return Path(args.output).open("w")


def main(args):
    with infile(args) as f:
        instance = Instance.parse(f.readlines())
        solver = SOLVERS[args.solver]
        solution = solver(instance)
        assert solution.valid()
        with outfile(args) as g:
            print("# Penalty: ", solution.penalty(), file=g)
            solution.serialize(g)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Solve a problem instance.")
    parser.add_argument("input", type=str, help="The input instance file to "
                        "read an instance from. Use - for stdin.")
    parser.add_argument("--solver", required=True, type=str,
                        help="The solver type.", choices=SOLVERS.keys())
    parser.add_argument("output", type=str,
                        help="The output file. Use - for stdout.",
                        default="-")
    main(parser.parse_args())
