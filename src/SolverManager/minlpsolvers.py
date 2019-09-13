from .isolver import ISolver
import pyomo.environ as pyo
import sys
import os

class CouenneSolver(ISolver):
    def __init__(self):
        exec_dir = sys.exec_prefix+'/bin'
        lib_dir = sys.exec_prefix+'/lib'
        exec_fpath = exec_dir + '/couenne'
        if "LD_LIBRARY_PATH" in os.environ:
            os.environ["LD_LIBRARY_PATH"] = f'{os.environ["LD_LIBRARY_PATH"]}:{exec_dir}:{lib_dir}'
        else:
            os.environ["LD_LIBRARY_PATH"] = f'{exec_dir}:{lib_dir}'
        
        self.solver = pyo.SolverFactory('couenne', executable = exec_fpath)