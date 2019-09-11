import pyomo.environ as pyo


def RouteConstraintsGenerator():
    def RouteConstraintsMaker(a_model):
        def ConstraintRoute(model, flow, node):
            sum_eq = - 1 if node == model.Src[flow] else + 1 if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node]) \
            == sum_eq
        a_model.ConstrRoute = pyo.Constraint(a_model.Flows, a_model.Nodes, rule = ConstraintRoute)
    return RouteConstraintsMaker


def NonlinearCapacityConstraintsGenerator():
    def NonlinearCapacityConstraintsMaker(a_model):
        def ConstraintCapacity(model, node_s, node_d):
            return sum(model.FlowStrain[flow] * model.FlowRoute[flow, node_s, node_d] for flow in model.Flows) \
            <= model.Capacity[node_s, node_d]
        a_model.ConstrCapacity = pyo.Constraint(a_model.Arcs, rule = ConstraintCapacity)
    return NonlinearCapacityConstraintsMaker


def LinearCapacityConstraintsGenerator():
    def LinearCapacityConstraintsMaker(a_model):

        a_model.FlowStrainMulRoute = pyo.Var(a_model.Flows, a_model.Arcs, domain = pyo.NonNegativeReals)

        def FlowStrainMulRouteConstraint1(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow] \
            >= 1.1*model.FlowUb*(model.FlowRoute[flow, node_s, node_d] - 1)
        a_model.FlowStrainMulRouteConstraint1 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint1)

        def FlowStrainMulRouteConstraint2(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            >= 0
        a_model.FlowStrainMulRouteConstraint2 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint2)

        def FlowStrainMulRouteConstraint3(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            <= model.FlowStrain[flow]
        a_model.FlowStrainMulRouteConstraint3 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint3)

        def FlowStrainMulRouteConstraint4(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            <= 1.1*model.FlowUb*model.FlowRoute[flow, node_s, node_d]
        a_model.FlowStrainMulRouteConstraint4 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint4)

        def ConstraintCapacity(model, node_s, node_d):
            return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows) \
            <= model.Capacity[node_s, node_d]
        a_model.ConstrCapacity = pyo.Constraint(a_model.Arcs, rule = ConstraintCapacity)
    return LinearCapacityConstraintsMaker


def ReformulatedConstraintsGenerator():
    def ReformulatedConstraintsMaker(a_model):

        a_model.FlowStrainMulRoute = pyo.Var(a_model.Flows, a_model.Arcs, domain = pyo.NonNegativeReals)
        
        def ConstraintRoute(model, flow, node):
            """In flow is equal to the out flow. """
            
            sum_eq = -model.FlowStrain[flow] if node == model.Src[flow] else model.FlowStrain[flow] if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowStrainMulRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowStrainMulRoute[flow, node, j] for j in model.NodesOut[node]) \
            == sum_eq
        a_model.ConstrRoute = pyo.Constraint(a_model.Flows, a_model.Nodes, rule = ConstraintRoute)

        def SingleFlowConstrain(model, flow, node):
            """Only one flow leaves the node. """
            
            sum_eq = 1 if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowRoute[flow, i, node] for i in model.NodesOut[node]) \
            <= 1
        a_model.ConstrSingleFlow = pyo.Constraint(a_model.Flows, a_model.Nodes, rule = SingleFlowConstrain)

        def FlowStrainMulRouteConstraint1(model, flow, node_s, node_d):
            """Flow_route help variable must be greate than 0 """

            return model.FlowStrainMulRoute[flow, node_s, node_d] >= 0
        a_model.FlowStrainMulRouteConstraint1 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint1)

        def FlowStrainMulRouteConstraint2(model, flow, node_s, node_d):
            """Flow_route help variable must be less than capacity if it flows via this edge. """

            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            <= model.Capacity[node_s, node_d] * model.FlowRoute[flow, node_s, node_d]
        a_model.FlowStrainMulRouteConstraint2 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint2)

    return ReformulatedConstraintsMaker