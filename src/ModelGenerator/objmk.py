import pyomo.environ as pyo

def LinearObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def LinearObjectiveMaker(a_model):
        def ObjectiveLin(model):
            return \
            sum((contin_weight/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) for flow in model.Flows) \
            - binary_weight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        a_model.Obj = pyo.Objective(rule = ObjectiveLin, sense = pyo.maximize)
    return LinearObjectiveMaker


def QuadObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def QuadObjectiveMaker(a_model):
        def ObjectiveQuad(model):
            return \
            sum((-contin_weight/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) ** 2 + 2 * (model.FlowStrain[flow] - model.FlowLb) for flow in model.Flows) \
             - binary_weight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        a_model.Obj = pyo.Objective(rule = ObjectiveQuad, sense = pyo.maximize)
    return QuadObjectiveMaker


def NonlinearObjectiveGenerator(alpha = 0.6, delta = 0.1, gamma = 1.2):
    def NonlinearObjectiveMaker(a_model):
        a_model.PortConnect = pyo.Var(a_model.Arcs, domain = pyo.Binary) 
        def ConstraintPortConnect(model, flow, node_s, node_d):
            return model.PortConnect[node_s, node_d] >= model.FlowRoute[flow, node_s, node_d]
        a_model.ConstrPortConnect = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = ConstraintPortConnect)
        def ObjectiveNonlinear(model):
            mu = 2 * gamma * (1 - alpha) * len(model.Arcs) / alpha
            epsilon = 1.3 * mu * model.FlowUb
            el = [ model.PortConnect[arc] for arc in model.Arcs ] 
            Qw = [ (-mu * model.FlowUb + epsilon) * (model.FlowStrain[flow] ** 2) / model.FlowUb ** 2 
                    + (mu * model.FlowUb - 2 * epsilon) * model.FlowStrain[flow] / model.FlowUb
                    + epsilon
                    for flow in model.Flows ]
            Blw = [ model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs ]
            return alpha * sum(Qw) + (1 - alpha) * (gamma * sum(el) + 2 * delta * sum(Blw))
        a_model.Obj = pyo.Objective(rule = ObjectiveNonlinear, sense = pyo.minimize)
    return NonlinearObjectiveMaker
