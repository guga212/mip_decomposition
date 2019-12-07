#changed constraints container continious model
sv_continious_constraint_rules_lhs = {}
sv_continious_constraint_rules_rhs = {}
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

#continious submodel help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS_c(model, flow, node_s, node_d):
    return -model.FlowStrainMulRoute[flow, node_s, node_d] + model.FlowStrain[flow]
def FlowStrainMulRouteConstraint1ExprRuleRHS_c(model, flow, node_s, node_d):
    return M_MULT * model.FlowUb
sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleLHS_c
sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleRHS_c

#binary submodel help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS_b(model, flow, node_s, node_d):
    if flow not in model.Flows:
         return 0
    else:
        return M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
sv_binary_constraint_rules_lhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleLHS_b
sv_binary_constraint_rules_rhs['FlowStrainMulRouteConstraint1'] = 0

#continious submodel help variable definition constraint 3
def FlowStrainMulRouteConstraint3ExprRuleLHS(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow]
sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint3'] = FlowStrainMulRouteConstraint3ExprRuleLHS
sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint3'] = 0

#continious submodel help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS_c(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d]
sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint4'] = FlowStrainMulRouteConstraint4ExprRuleLHS_c
sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint4'] = 0

#binary submodel help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS_b(model, flow, node_s, node_d):
    if flow not in model.Flows:
         return 0
    else:
        return -M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
sv_binary_constraint_rules_lhs['FlowStrainMulRouteConstraint4'] = FlowStrainMulRouteConstraint4ExprRuleLHS_b
sv_binary_constraint_rules_rhs['FlowStrainMulRouteConstraint4'] = 0

#continious submodel linear capacity constraint
def ConstraintCapacityLinearExprRuleLHS(model, node_s, node_d):
    return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows )
def ConstraintCapacityLinearExprRuleRHS(model, node_s, node_d):
    return model.Capacity[node_s, node_d]
sv_continious_constraint_rules_lhs['CapacityConstraintLinear'] = ConstraintCapacityLinearExprRuleLHS
sv_continious_constraint_rules_rhs['CapacityConstraintLinear'] = ConstraintCapacityLinearExprRuleRHS