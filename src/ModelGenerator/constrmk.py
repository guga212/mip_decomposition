import pyomo.environ as pyo
import operator as op

Comparsions = {
    '==': op.eq,
    '<=': op.le,
    }

def AddConstraint(amodel, constr_name, expr_rule_lhs, expr_rule_rhs, comp_op, *constr_sets_names):
    if callable(expr_rule_rhs):
        def ConstraintRule(model, *args):
            return Comparsions[comp_op](expr_rule_lhs(model, *args), expr_rule_rhs(model, *args))
    else:
        expr_rhs_value = expr_rule_rhs
        def expr_rhs_proxy(*args):
            return expr_rhs_value
        expr_rule_rhs = expr_rhs_proxy
        def ConstraintRule(model, *args):
            return Comparsions[comp_op](expr_rule_lhs(model, *args), expr_rhs_value)
    
    constr_sets = [getattr(amodel, name) for name in constr_sets_names] 
    constraint = pyo.Constraint(*constr_sets, rule = ConstraintRule, name = constr_name)
    setattr(amodel, constr_name, constraint)

    model_constraint = getattr(amodel, constr_name)
    amodel.Suffix[model_constraint] = { 'LHS': expr_rule_lhs, 'RHS': expr_rule_rhs, 'SetsNames': constr_sets_names,
                                        'CompareName': comp_op, 'CompareOperation': Comparsions[comp_op] }


def RouteConstraintsGenerator():
    def RouteConstraintsMaker(amodel):
        def ConstraintRouteExprRuleLHS(model, flow, node):
            return 0 \
            + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node])
        def ConstraintRouteExprRuleRHS(model, flow, node):
            sum_eq = -1 if node == model.Src[flow] else 1 if node == model.Dst[flow] else 0
            return sum_eq
        AddConstraint(amodel, 'RouteConstraint', ConstraintRouteExprRuleLHS, 
                        ConstraintRouteExprRuleRHS, '==', 'Flows', 'Nodes')
    return RouteConstraintsMaker


def NonlinearCapacityConstraintsGenerator():
    def NonlinearCapacityConstraintsMaker(amodel):
        def ConstraintCapacityExprRuleLHS(model, node_s, node_d):
            return sum(model.FlowStrain[flow] * model.FlowRoute[flow, node_s, node_d] for flow in model.Flows)
        def ConstraintCapacityExprRuleRHS(model, node_s, node_d):
            return model.Capacity[node_s, node_d]
        AddConstraint(amodel, 'CapacityConstraint', ConstraintCapacityExprRuleLHS, 
                        ConstraintCapacityExprRuleRHS, '<=', 'Arcs')
    return NonlinearCapacityConstraintsMaker


def LinearCapacityConstraintsGenerator():
    def LinearCapacityConstraintsMaker(amodel):

        #multiplier to achive value greater than any posiible flow
        M_MULT = 1.1

        amodel.FlowStrainMulRoute = pyo.Var(amodel.FlowRoute.index_set(), domain = pyo.NonNegativeReals)

        def FlowStrainMulRouteConstraint1ExprRuleLHS(model, flow, node_s, node_d):
            """Help variable is greater or equal to the flow if the route exist right hand value. """
            return -model.FlowStrainMulRoute[flow, node_s, node_d] + model.FlowStrain[flow] \
                    + M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
        def FlowStrainMulRouteConstraint1ExprRuleRHS(model, flow, node_s, node_d):
            """Help variable is greater or equal to the flow if the route exist left hand value. """
            return M_MULT * model.FlowUb
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint1', FlowStrainMulRouteConstraint1ExprRuleLHS,
                        FlowStrainMulRouteConstraint1ExprRuleRHS, '<=', 'Flows', 'Arcs')

        def FlowStrainMulRouteConstraint3ExprRuleLHS(model, flow, node_s, node_d):
            """Help variable is less or equal to the flow. """
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow]
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint3', FlowStrainMulRouteConstraint3ExprRuleLHS, 
                        0, '<=', 'Flows', 'Arcs')

        def FlowStrainMulRouteConstraint4ExprRuleLHS(model, flow, node_s, node_d):
            """Help variable is less or equal zero if the route doesn't exitst. """
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
                    - M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint4', FlowStrainMulRouteConstraint4ExprRuleLHS, 
                        0, '<=', 'Flows', 'Arcs')

        def ConstraintCapacityExprRuleLHS(model, node_s, node_d):
            """Sum of the all flows less or equal to the capacity left hand value"""
            return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows )
        def ConstraintCapacityExprRuleRHS(model, node_s, node_d):
            """Sum of the all flows less or equal to the capacity right hand value"""
            return model.Capacity[node_s, node_d]
        AddConstraint(amodel, 'CapacityConstraintLinear', ConstraintCapacityExprRuleLHS, 
                        ConstraintCapacityExprRuleRHS, '<=', 'Arcs')

    return LinearCapacityConstraintsMaker


def ReformulatedConstraintsGenerator():
    def ReformulatedConstraintsMaker(amodel):
        """
        Binary route variables for this formualation 
        are unable to force a flow, but they are able 
        to prevent a flow.
        """
        
        amodel.FlowStrainMulRoute = pyo.Var(amodel.FlowRoute.index_set(), domain = pyo.NonNegativeReals)
        
        def ConstraintRouteExprRuleLHS(model, flow, node):
            """In flow is equal to the out flow left hand value. """            
            return 0 \
            + sum(model.FlowStrainMulRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowStrainMulRoute[flow, node, j] for j in model.NodesOut[node])            
        def ConstraintRouteExprRuleRHS(model, flow, node):
            """In flow is equal to the out flow left right value. """
            sum_eq = -model.FlowStrain[flow] if node == model.Src[flow] else model.FlowStrain[flow] if node == model.Dst[flow] else 0
            return sum_eq
        AddConstraint(amodel, 'RouteConstraintContiniousRef', ConstraintRouteExprRuleLHS,
                        ConstraintRouteExprRuleRHS, '==', 'Flows', 'Nodes')

        def SingleFlowConstraintRuleExprLHS(model, flow, node):
            """Only one flow leaves the node. """
            return sum(model.FlowRoute[flow, node, i] for i in model.NodesOut[node])
        AddConstraint(amodel, 'SingleFlowConstraintRef', SingleFlowConstraintRuleExprLHS,
                        1, '<=', 'Flows', 'Nodes')

        def FlowStrainMulRouteConstraint1ExprRuleLHS(model, flow, node_s, node_d):
            """Flow_route help variable must be less than capacity if it flows via this edge. """
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            - model.Capacity[node_s, node_d] * model.FlowRoute[flow, node_s, node_d]
        AddConstraint(amodel, 'FlowStrainMulRouteConstraint1Ref', FlowStrainMulRouteConstraint1ExprRuleLHS, 
                        0, '<=', 'Flows', 'Arcs')

        def ConstraintCapacityExprRuleLHS(model, node_s, node_d):
            """Sum of the help variables less or equal to the capacity left hand value"""
            return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows )
        def ConstraintCapacityExprRuleRHS(model, node_s, node_d):
            """Sum of the help variables less or equal to the capacity right hand value"""
            return model.Capacity[node_s, node_d]
        AddConstraint(amodel, 'CapacityConstraintRef', ConstraintCapacityExprRuleLHS, 
                        ConstraintCapacityExprRuleRHS, '<=', 'Arcs')

    return ReformulatedConstraintsMaker