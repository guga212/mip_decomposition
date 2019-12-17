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
            -model.FlowStrainWeight*sum((model.FlowStrain[flow] - model.FlowLb) ** 2 for flow in model.Flows) \
            + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        amodel.Obj = pyo.Objective(rule = ObjectiveQuad, sense = pyo.minimize)
        amodel.Suffix[amodel.Obj] = 'Quadratic'
    return QuadObjectiveMaker


def LogObjectiveGenerator(binary_weight = 0.05, contin_weight = 1.0):
    def LogObjectiveMaker(amodel):
        amodel.FlowStrainWeight = pyo.Param(mutable = True, within = pyo.PositiveReals, default = contin_weight)
        amodel.FlowRouteWeight = pyo.Param(mutable = True, within = pyo.PositiveReals, default = binary_weight)
        def ObjectiveLog(model):
            ro = 0.1
            return 0\
            -model.FlowStrainWeight*sum(pyo.log(ro + model.FlowStrain[flow]) for flow in model.Flows ) \
            + model.FlowRouteWeight*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        amodel.Obj = pyo.Objective(rule = ObjectiveLog, sense = pyo.minimize)
        amodel.Suffix[amodel.Obj] = 'Logarithmic'
    return LogObjectiveMaker
