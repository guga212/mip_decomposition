import pyomo.environ as pyo

def ObjectiveBin(model):
    return 0 \
    + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)

def ObjectiveContLin(model):
    return 0 \
    - model.FlowStrainWeight*sum((1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) 
                                    for flow in model.Flows)

def ObjectiveContQuad(model):
    return 0\
    -model.FlowStrainWeight*sum((-1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) ** 2 + 
                                    2 * ( (model.FlowStrain[flow] - model.FlowLb) / (model.FlowUb - model.FlowLb))
                                    for flow in model.Flows)

def ObjectiveContLog(model):
    return 0 \
    - model.FlowStrainWeight*sum(pyo.log(model.FlowStrain[flow]/(model.FlowUb - model.FlowLb)) for flow in model.Flows)