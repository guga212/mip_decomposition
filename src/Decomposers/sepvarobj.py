import pyomo.environ as pyo
sv_objectives = {}

def ObjectiveBin(model):
    return 0 \
    + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
sv_objectives['Route'] = ObjectiveBin

def ObjectiveContLin(model):
    return 0 \
    - model.FlowStrainWeight*sum((1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) 
                                    for flow in model.Flows)
sv_objectives['Linear'] = ObjectiveContLin

def ObjectiveContQuad(model):
    return 0\
    -model.FlowStrainWeight*sum((-1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) ** 2 + 
                                    2 * ( (model.FlowStrain[flow] - model.FlowLb) / (model.FlowUb - model.FlowLb))
                                    for flow in model.Flows)
sv_objectives['Quadratic'] = ObjectiveContQuad

def ObjectiveContLog(model):
    return 0 \
    - model.FlowStrainWeight*sum(pyo.log(model.FlowStrain[flow]/(model.FlowUb - model.FlowLb)) for flow in model.Flows)
sv_objectives['Logarithmic'] = ObjectiveContLog