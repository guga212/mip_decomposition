import pyomo.environ as pyo

def ObjectiveBinLin(model):
    return 0 \
    + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)

def ObjectiveBinQuad(model):
    return 0\
    + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)