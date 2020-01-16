from .isolver import ISolver
import pyomo.environ as pyo
import sys

class GlpkSolver(ISolver):
    def __init__(self):
        super().__init__()
        exec_dir = sys.exec_prefix+'/bin'
        exec_fpath = exec_dir + '/glpsol'
        self.solver = pyo.SolverFactory('glpk', executable = exec_fpath)