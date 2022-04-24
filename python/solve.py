"""Solves an instance.

Modify this file to implement your own solvers.

For usage, run `python3 solve.py --help`.
"""

import argparse
from math import sqrt
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


def create_data_model(instance: Instance):
    data = {}
    d2 = instance.D**2
    data["num_vars"] = d2
    data["num_constraints"] = instance.N
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


def solve_cover(instance: Instance) -> Solution:
    data = create_data_model(instance)
    solver = pywraplp.Solver.CreateSolver("SCIP")

    x = {}
    for j in range(data["num_vars"]):
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
