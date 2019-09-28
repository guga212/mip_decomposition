import pyomo.environ as pyo
import operator as op

Comparsions = {
    '==': op.eq,
    '<=': op.le,
    }


def AddConstraint(amodel, constr_name, expr_rule, comp_op, *constr_sets):
    def ConstraintRule(model, *args):
        return Comparsions[comp_op](expr_rule(model, *args), 0)
    constraint = pyo.Constraint(*constr_sets, rule = ConstraintRule, name = constr_name)
    setattr(amodel, constr_name, constraint)

    model_constraint = getattr(amodel, constr_name)
    amodel.Suffix[model_constraint] = (expr_rule, comp_op)


def RouteConstraintsGenerator():
    def RouteConstraintsMaker(amodel):
        def ConstraintRouteExprRule(model, flow, node):
            sum_eq = -1 if node == model.Src[flow] else 1 if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node]) \
            - sum_eq                        
        AddConstraint(amodel, 'RouteConstraint', ConstraintRouteExprRule, '==', amodel.Flows, amodel.Nodes)
    return RouteConstraintsMaker


def NonlinearCapacityConstraintsGenerator():
    def NonlinearCapacityConstraintsMaker(amodel):
        def ConstraintCapacityExprRule(model, node_s, node_d):
            return sum(model.FlowStrain[flow] * model.FlowRoute[flow, node_s, node_d] for flow in model.Flows) \
            - model.Capacity[node_s, node_d]
        amodel.ConstraintCapacityExprRule = ConstraintCapacityExprRule
        amodel.ConstrCapacityCompOp = '<='
        AddConstraint(amodel, 'CapacityConstraint', ConstraintCapacityExprRule, '<=', amodel.Arcs)
    return NonlinearCapacityConstraintsMaker


def LinearCapacityConstraintsGenerator():
    def LinearCapacityConstraintsMaker(amodel):

        #multiplier to achive value greater than any posiible flow
        M_MULT = 1.1

        amodel.FlowStrainMulRoute = pyo.Var(amodel.Flows, amodel.Arcs, domain = pyo.NonNegativeReals)

        def FlowStrainMulRouteConstraint1ExprRule(model, flow, node_s, node_d):
            """Help variable is greater or equal to the flow if the route exist. """
            return -model.FlowStrainMulRoute[flow, node_s, node_d] + model.FlowStrain[flow] \
                    + M_MULT * model.FlowUb * (model.FlowRoute[flow, node_s, node_d] - 1)
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint1', FlowStrainMulRouteConstraint1ExprRule, '<=', amodel.Flows, amodel.Arcs)

        def FlowStrainMulRouteConstraint3ExprRule(model, flow, node_s, node_d):
            """Help variable is less or equal to the flow. """
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow]
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint3', FlowStrainMulRouteConstraint3ExprRule, '<=', amodel.Flows, amodel.Arcs)

        def FlowStrainMulRouteConstraint4ExprRule(model, flow, node_s, node_d):
            """Help variable is less or equal zero if the route doesn't exitst. """
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
                    - M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint4', FlowStrainMulRouteConstraint4ExprRule, '<=', amodel.Flows, amodel.Arcs)

        def ConstraintCapacityExprRule(model, node_s, node_d):
            """Sum of the all flows less or equal to the capacity"""
            return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows) \
            - model.Capacity[node_s, node_d]
        AddConstraint(amodel, 'CapacityConstraint', ConstraintCapacityExprRule, '<=', amodel.Arcs)

    return LinearCapacityConstraintsMaker


def ReformulatedConstraintsGenerator():
    def ReformulatedConstraintsMaker(amodel):
        """
        Binary route variables for this formualation are 
        unable to force a flow, but the are able to prevent the flow.
        """
        
        amodel.FlowStrainMulRoute = pyo.Var(amodel.Flows, amodel.Arcs, domain = pyo.NonNegativeReals)
        
        def ConstraintRouteExprRule(model, flow, node):
            """In flow is equal to the out flow. """
            sum_eq = -model.FlowStrain[flow] if node == model.Src[flow] else model.FlowStrain[flow] if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowStrainMulRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowStrainMulRoute[flow, node, j] for j in model.NodesOut[node]) \
            - sum_eq
        AddConstraint(amodel, 'RouteConstraint', ConstraintRouteExprRule, '==', amodel.Flows, amodel.Nodes)

        def SingleFlowConstraintRuleExpr(model, flow, node):
            """Only one flow leaves the node. """
            return sum(model.FlowRoute[flow, node, i] for i in model.NodesOut[node]) - 1
        AddConstraint(amodel, 'SingleFlowConstraint', SingleFlowConstraintRuleExpr, '<=', amodel.Flows, amodel.Nodes)

        def FlowStrainMulRouteConstraint2Expr(model, flow, node_s, node_d):
            """Flow_route help variable must be less than capacity if it flows via this edge. """
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            - model.Capacity[node_s, node_d] * model.FlowRoute[flow, node_s, node_d]
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint2', FlowStrainMulRouteConstraint2Expr, '<=', amodel.Flows, amodel.Arcs)

    return ReformulatedConstraintsMaker