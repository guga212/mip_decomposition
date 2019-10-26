#changed constraints container binary mode
sv_binary_constraint_rules_lhs = {}
sv_binary_constraint_rules_rhs = {}
M_MULT = 1.1

#binary submodel route constraint
def ConstraintRouteExprRuleLHS(model, flow, node):
    return 0 \
    + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
    - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node])
def ConstraintRouteExprRuleRHS(model, flow, node):
    sum_eq = -1 if node == model.Src[flow] else 1 if node == model.Dst[flow] else 0
    return sum_eq
sv_binary_constraint_rules_lhs['RouteConstraint'] = ConstraintRouteExprRuleLHS
sv_binary_constraint_rules_rhs['RouteConstraint'] = ConstraintRouteExprRuleRHS

#binary submodel help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS(model, flow, node_s, node_d):
    return M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
sv_binary_constraint_rules_lhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleLHS
sv_binary_constraint_rules_rhs['FlowStrainMulRouteConstraint1'] = 0

#binary submodel help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS(model, flow, node_s, node_d):
    return -M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
sv_binary_constraint_rules_lhs['FlowStrainMulRouteConstraint4'] = FlowStrainMulRouteConstraint4ExprRuleLHS
sv_binary_constraint_rules_rhs['FlowStrainMulRouteConstraint4'] = 0