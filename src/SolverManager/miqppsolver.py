from .isolver import ISolver
import pyomo.environ as pyo
import sys

class CplexSolver(ISolver):
    def __init__(self):
        super().__init__()
        exec_dir = sys.exec_prefix+'/bin'
        exec_fpath = exec_dir + '/cplex'
        self.solver = pyo.SolverFactory('cplex', executable = exec_fpath)
        self.solver.options["timelimit"] = self.timeout