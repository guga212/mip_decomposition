import pyomo.environ as pyo

def LinearObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def LinearObjectiveMaker(a_model):
        def ObjectiveLin(model):
            return \
            sum((contin_weight/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) for flow in model.Flows) + \
            -binary_weight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        a_model.Obj = pyo.Objective(rule = ObjectiveLin, sense = pyo.maximize)
    return LinearObjectiveMaker


def QuadObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def QuadObjectiveMaker(a_model):
        def ObjectiveQuad(model):
            return \
            sum((contin_weight/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) ** 2 + 2 * (model.FlowStrain[flow] - model.FlowLb)   for flow in model.Flows) + \
            -binary_weight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        a_model.Obj = pyo.Objective(rule = ObjectiveQuad, sense = pyo.maximize)
    return QuadObjectiveMaker
