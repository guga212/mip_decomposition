from .isolver import ISolver
import pyomo.environ as pyo
import sys

class IpoptSolver(ISolver):
    def __init__(self):
        exec_dir = sys.exec_prefix+'/bin'
        exec_fpath = exec_dir + '/ipopt'
        self.solver = pyo.SolverFactory('ipopt', executable = exec_fpath)