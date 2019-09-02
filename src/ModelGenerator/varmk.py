from pyomo.environ import *

def BinRouteContFlow(rs_model):
    rs_model.model.FlowStrain = Var(rs_model.model.Flows, domain = NonNegativeReals)
    rs_model.model.FlowRoute = Var(rs_model.model.Flows, rs_model.model.Arcs, domain = Binary)