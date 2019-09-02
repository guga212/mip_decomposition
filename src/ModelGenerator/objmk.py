from pyomo.environ import *

def LinearObjective(rs_model):
    rs_model.model.objective = None

