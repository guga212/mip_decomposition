import pyomo.environ as pyo
import operator as op

Comparsions = {
    "==": op.eq,
    ">=": op.ge,
    "<=": op.le,
    ">" : op.gt,
    "<" : op.gt
    }

def RouteConstraintsGenerator():
    def RouteConstraintsMaker(a_model):
        def ConstraintRouteExprRule(model, flow, node):
            sum_eq = - 1 if node == model.Src[flow] else + 1 if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node]) \
            - sum_eq
        a_model.ConstraintRouteExprRule = ConstraintRouteExprRule
        a_model.ConstrRouteCompOp = "=="
        def ConstraintRouteRule(model, flow, node):
            return Comparsions[a_model.ConstrRouteCompOp](a_model.ConstraintRouteExprRule(model, flow, node), 0)
        a_model.ConstrRoute = pyo.Constraint(a_model.Flows, a_model.Nodes, rule = ConstraintRouteRule)
    return RouteConstraintsMaker


def NonlinearCapacityConstraintsGenerator():
    def NonlinearCapacityConstraintsMaker(a_model):
        def ConstraintCapacityExprRule(model, node_s, node_d):
            return sum(model.FlowStrain[flow] * model.FlowRoute[flow, node_s, node_d] for flow in model.Flows) \
            - model.Capacity[node_s, node_d]
        a_model.ConstraintCapacityExprRule = ConstraintCapacityExprRule
        a_model.ConstrCapacityCompOp = "<="
        def ConstraintCapacityRule(model, node_s, node_d):
            return Comparsions[a_model.ConstrCapacityCompOp](a_model.ConstraintCapacityExprRule(model, node_s, node_d), 0)
        a_model.ConstrCapacity = pyo.Constraint(a_model.Arcs, rule = ConstraintCapacityRule)
    return NonlinearCapacityConstraintsMaker


def LinearCapacityConstraintsGenerator():
    def LinearCapacityConstraintsMaker(a_model):

        #multiplier to achive value greater than any posiible flow
        M_MULT = 1.1

        a_model.FlowStrainMulRoute = pyo.Var(a_model.Flows, a_model.Arcs, domain = pyo.NonNegativeReals)

        def FlowStrainMulRouteConstraint1ExprRule(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow] \
                    - M_MULT * model.FlowUb*(model.FlowRoute[flow, node_s, node_d] - 1)
        a_model.FlowStrainMulRouteConstraint1ExprRule = FlowStrainMulRouteConstraint1ExprRule
        a_model.FlowStrainMulRouteConstraint1CompOp = ">="
        def FlowStrainMulRouteConstraint1Rule(model, flow, node_s, node_d):
            """Help variable greater or equal to the flow if route exitst otherwise greater or equal to the minus upper flow value. """
            return Comparsions[a_model.FlowStrainMulRouteConstraint1CompOp](a_model.FlowStrainMulRouteConstraint1ExprRule(model, flow, node_s, node_d), 0)
        a_model.FlowStrainMulRouteConstraint1 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint1Rule)

        def FlowStrainMulRouteConstraint3ExprRule(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow] \
                    - model.FlowStrain[flow]
        a_model.FlowStrainMulRouteConstraint3ExprRule = FlowStrainMulRouteConstraint3ExprRule
        a_model.FlowStrainMulRouteConstraint3CompOp = "<="
        def FlowStrainMulRouteConstraint3Rule(model, flow, node_s, node_d):
            """Help variable less or equal to the flow. """
            return Comparsions[a_model.FlowStrainMulRouteConstraint3CompOp](a_model.FlowStrainMulRouteConstraint3ExprRule(model, flow, node_s, node_d), 0)
        a_model.FlowStrainMulRouteConstraint3 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint3Rule)

        def FlowStrainMulRouteConstraint4ExprRule(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow] \
                    - M_MULT * model.FlowUb*model.FlowRoute[flow, node_s, node_d]
        a_model.FlowStrainMulRouteConstraint4ExprRule = FlowStrainMulRouteConstraint4ExprRule
        a_model.FlowStrainMulRouteConstraint4CompOp = "<="
        def FlowStrainMulRouteConstraint4Rule(model, flow, node_s, node_d):
            """Help variable less or equal to the upper flow if route exitst otherwise less or equal to the. """
            return Comparsions[a_model.FlowStrainMulRouteConstraint4CompOp](a_model.FlowStrainMulRouteConstraint4ExprRule(model, flow, node_s, node_d), 0)
        a_model.FlowStrainMulRouteConstraint4 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint4Rule)

        def ConstraintCapacityExprRule(model, node_s, node_d):
            return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows) \
            - model.Capacity[node_s, node_d]
        a_model.ConstraintCapacityExprRule = ConstraintCapacityExprRule
        a_model.ConstrCapacityCompOp = "<="
        def ConstraintCapacityRule(model, node_s, node_d):
            """Sum of the all flows less or equal to the capacity"""
            return Comparsions[a_model.ConstrCapacityCompOp](a_model.ConstraintCapacityExprRule(model, node_s, node_d), 0)
        a_model.ConstrCapacity = pyo.Constraint(a_model.Arcs, rule = ConstraintCapacityRule)
    return LinearCapacityConstraintsMaker


def ReformulatedConstraintsGenerator():
    def ReformulatedConstraintsMaker(a_model):

        a_model.FlowStrainMulRoute = pyo.Var(a_model.Flows, a_model.Arcs, domain = pyo.NonNegativeReals)
        
        def ConstraintRouteExprRule(model, flow, node):
            sum_eq = -model.FlowStrain[flow] if node == model.Src[flow] else model.FlowStrain[flow] if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowStrainMulRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowStrainMulRoute[flow, node, j] for j in model.NodesOut[node]) \
            - sum_eq
        a_model.ConstraintRouteExprRule = ConstraintRouteExprRule
        a_model.ConstrRouteCompOp = "=="        
        def ConstraintRouteRule(model, flow, node):
            """In flow is equal to the out flow. """            
            return Comparsions[a_model.ConstrRouteCompOp](a_model.ConstraintRouteExprRule(model, flow, node), 0)
        a_model.ConstrRoute = pyo.Constraint(a_model.Flows, a_model.Nodes, rule = ConstraintRouteRule)

        def SingleFlowConstrainRuleExpr(model, flow, node):
            return sum(model.FlowRoute[flow, node, i] for i in model.NodesOut[node]) - 1
        a_model.SingleFlowConstrainRuleExpr = SingleFlowConstrainRuleExpr
        a_model.SingleFlowConstrainRuleCompOp = "<="
        def SingleFlowConstrainRule(model, flow, node):
            """Only one flow leaves the node. """
            return Comparsions[a_model.SingleFlowConstrainRuleCompOp](a_model.SingleFlowConstrainRuleExpr(model, flow, node), 0)
        a_model.ConstrSingleFlow = pyo.Constraint(a_model.Flows, a_model.Nodes, rule = SingleFlowConstrainRule)

        def FlowStrainMulRouteConstraint2Expr(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            - model.Capacity[node_s, node_d] * model.FlowRoute[flow, node_s, node_d]
        a_model.FlowStrainMulRouteConstraint2Expr = FlowStrainMulRouteConstraint2Expr
        a_model.FlowStrainMulRouteConstraint2CompOp = "<="       
        def FlowStrainMulRouteConstraint2Rule(model, flow, node_s, node_d):
            """Flow_route help variable must be less than capacity if it flows via this edge. """
            return Comparsions[a_model.FlowStrainMulRouteConstraint2CompOp](a_model.FlowStrainMulRouteConstraint2Expr(model, flow, node_s, node_d), 0)
        a_model.FlowStrainMulRouteConstraint2 = pyo.Constraint(a_model.Flows, a_model.Arcs, rule = FlowStrainMulRouteConstraint2Rule)

    return ReformulatedConstraintsMaker