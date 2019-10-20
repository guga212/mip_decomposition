import pyomo.environ as pyo

def ObjectiveLin(model):
    return 0 \
    - model.FlowStrainWeight*sum((1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow, subnet] - model.FlowLb) for flow in model.Flows for subnet in model.Subnets) \
    + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.OriginalArcs)

def ObjectiveQuad(model):
    return 0\
    -model.FlowStrainWeight*sum((-1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow, subnet] - model.FlowLb) ** 2 + 
                                    2 * ( (model.FlowStrain[flow, subnet] - model.FlowLb) / (model.FlowUb - model.FlowLb))
                                    for flow in model.Flows for subnet in model.Subnets) \
    + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.OriginalArcs)
