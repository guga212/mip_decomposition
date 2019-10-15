import pyomo.environ as pyo

def LinearObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def LinearObjectiveMaker(amodel):
        amodel.FlowStrainWeight = pyo.Param(mutable = True, within = pyo.PositiveReals, default = contin_weight)
        amodel.FlowRouteWeight = pyo.Param(mutable = True, within = pyo.PositiveReals, default = binary_weight)
        def ObjectiveLin(model):
            return 0 \
            - model.FlowStrainWeight*sum((1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) for flow in model.Flows) \
            + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        amodel.Obj = pyo.Objective(rule = ObjectiveLin, sense = pyo.minimize)
        amodel.Suffix[amodel.Obj] = 'Linear'
    return LinearObjectiveMaker


def QuadObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def QuadObjectiveMaker(amodel):
        amodel.FlowStrainWeight = pyo.Param(mutable = True, within = pyo.PositiveReals, default = contin_weight)
        amodel.FlowRouteWeight = pyo.Param(mutable = True, within = pyo.PositiveReals, default = binary_weight)
        def ObjectiveQuad(model):
            return 0\
            -model.FlowStrainWeight*sum((-1/(model.FlowUb - model.FlowLb)) * (model.FlowStrain[flow] - model.FlowLb) ** 2 + 
                                        2 * ( (model.FlowStrain[flow] - model.FlowLb) / (model.FlowUb - model.FlowLb))
                                        for flow in model.Flows) \
            + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        amodel.Obj = pyo.Objective(rule = ObjectiveQuad, sense = pyo.minimize)
        amodel.Suffix[amodel.Obj] = 'Quadratic'
    return QuadObjectiveMaker


# def NonlinearObjectiveGenerator(alpha = 0.6, delta = 0.1, gamma = 1.2):
#     def NonlinearObjectiveMaker(amodel):
#         amodel.PortConnect = pyo.Var(amodel.Arcs, domain = pyo.Binary) 
#         def ConstraintPortConnect(model, flow, node_s, node_d):
#             return model.PortConnect[node_s, node_d] >= model.FlowRoute[flow, node_s, node_d]
#         amodel.ConstrPortConnect = pyo.Constraint(amodel.Flows, amodel.Arcs, rule = ConstraintPortConnect)
#         def ObjectiveNonlinear(model):
#             mu = 2 * gamma * (1 - alpha) * len(model.Arcs) / alpha
#             epsilon = 1.3 * mu * model.FlowUb
#             el = [ model.PortConnect[arc] for arc in model.Arcs ] 
#             Qw = [ (-mu * model.FlowUb + epsilon) * (model.FlowStrain[flow] ** 2) / model.FlowUb ** 2 
#                     + (mu * model.FlowUb - 2 * epsilon) * model.FlowStrain[flow] / model.FlowUb
#                     + epsilon
#                     for flow in model.Flows ]
#             Blw = [ model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs ]
#             return alpha * sum(Qw) + (1 - alpha) * (gamma * sum(el) + 2 * delta * sum(Blw))
#         amodel.Obj = pyo.Objective(rule = ObjectiveNonlinear, sense = pyo.minimize)
#     return NonlinearObjectiveMaker
