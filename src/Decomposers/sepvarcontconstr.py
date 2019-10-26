#changed constraints container continious model
sv_continious_constraint_rules_lhs = {}
sv_continious_constraint_rules_rhs = {}
M_MULT = 1.1

#continious submodel help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS(model, flow, node_s, node_d):
    return -model.FlowStrainMulRoute[flow, node_s, node_d] + model.FlowStrain[flow]
def FlowStrainMulRouteConstraint1ExprRuleRHS(model, flow, node_s, node_d):
    return M_MULT * model.FlowUb
sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleLHS
sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleRHS

#continious submodel help variable definition constraint 3
def FlowStrainMulRouteConstraint3ExprRuleLHS(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow]
sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint3'] = FlowStrainMulRouteConstraint3ExprRuleLHS
sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint3'] = 0

#continious submodel help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d]
sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint4'] = FlowStrainMulRouteConstraint4ExprRuleLHS
sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint4'] = 0

#continious submodel linear capacity constraint
def ConstraintCapacityLinearExprRuleLHS(model, node_s, node_d):
    return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows )
def ConstraintCapacityLinearExprRuleRHS(model, node_s, node_d):
    return model.Capacity[node_s, node_d]
sv_continious_constraint_rules_lhs['CapacityConstraintLinear'] = ConstraintCapacityLinearExprRuleLHS
sv_continious_constraint_rules_rhs['CapacityConstraintLinear'] = ConstraintCapacityLinearExprRuleRHS