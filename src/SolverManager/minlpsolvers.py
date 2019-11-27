from .isolver import ISolver
import pyomo.environ as pyo
import sys

class CouenneSolver(ISolver):
    def __init__(self):
        exec_dir = sys.exec_prefix+'/bin'
        exec_fpath = exec_dir + '/couenne'
        self.solver = pyo.SolverFactory('couenne', executable = exec_fpath)

class AslBaronSolver(ISolver):
    def __init__(self):
        exec_dir = sys.exec_prefix+'/bin'
        exec_fpath = exec_dir + '/baron'
        self.solver = pyo.SolverFactory('asl:baron', executable = exec_fpath)