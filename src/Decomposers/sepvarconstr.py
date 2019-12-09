
#changed constraints container continious model
sv_continious_constraint_rules = {}
#changed constraints container binary mode
sv_binary_constraint_rules = {}
#big value parameter
M_MULT = 1.1
#utility functions
def AddConstraint(constraints_keeper, key, lhs, rhs, sign, *sets):
    constraints_keeper[key] = {'lhs': lhs, 'rhs': rhs, 'sign': sign, 'sets': sets}
def AddConstraintContinious(key, lhs, rhs, sign, *sets):
    AddConstraint(sv_continious_constraint_rules, key, lhs, rhs, sign, *sets)
def AddConstraintBinary(key, lhs, rhs, sign, *sets):
    AddConstraint(sv_binary_constraint_rules, key, lhs, rhs, sign, *sets)

#binary submodel route constraint
def ConstraintRouteExprRuleLHS(model, flow, node):
    return 0 \
    + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
    - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node])
def ConstraintRouteExprRuleRHS(model, flow, node):
    sum_eq = -1 if node == model.Src[flow] else 1 if node == model.Dst[flow] else 0
    return sum_eq
AddConstraintBinary('RouteConstraint', ConstraintRouteExprRuleLHS, ConstraintRouteExprRuleRHS, '==', 'Flows', 'Nodes')

#continious submodel help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS_c(model, flow, node_s, node_d):
    return -model.FlowStrainMulRoute[flow, node_s, node_d] + model.FlowStrain[flow]
def FlowStrainMulRouteConstraint1ExprRuleRHS_c(model, flow, node_s, node_d):
    return M_MULT * model.FlowUb
AddConstraintContinious('FlowStrainMulRouteConstraint1', FlowStrainMulRouteConstraint1ExprRuleLHS_c, 
                                                        FlowStrainMulRouteConstraint1ExprRuleRHS_c,
                                                        '<=', 'Flows', 'Arcs')

#binary submodel help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS_b(model, flow, node_s, node_d):
    if flow not in model.Flows:
         return 0
    else:
        return M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
AddConstraintBinary('FlowStrainMulRouteConstraint1', FlowStrainMulRouteConstraint1ExprRuleLHS_b, 0, '<=', 'Flows', 'Arcs')

#continious submodel help variable definition constraint 3
def FlowStrainMulRouteConstraint3ExprRuleLHS(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow]
AddConstraintContinious('FlowStrainMulRouteConstraint3', FlowStrainMulRouteConstraint3ExprRuleLHS, 0, '<=', 'Flows', 'Arcs')

#continious submodel help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS_c(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d]
AddConstraintContinious('FlowStrainMulRouteConstraint4', FlowStrainMulRouteConstraint4ExprRuleLHS_c, 0, '<=', 'Flows', 'Arcs')

#binary submodel help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS_b(model, flow, node_s, node_d):
    if flow not in model.Flows:
         return 0
    else:
        return -M_MULT * model.FlowUb * model.FlowRoute[flow, node_s, node_d]
AddConstraintBinary('FlowStrainMulRouteConstraint4', FlowStrainMulRouteConstraint4ExprRuleLHS_b, 0, '<=', 'Flows', 'Arcs')

#continious submodel linear capacity constraint
def ConstraintCapacityLinearExprRuleLHS(model, node_s, node_d):
    return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows )
def ConstraintCapacityLinearExprRuleRHS(model, node_s, node_d):
    return model.Capacity[node_s, node_d]
AddConstraintContinious('CapacityConstraintLinear', ConstraintCapacityLinearExprRuleLHS, 
                                                    ConstraintCapacityLinearExprRuleRHS,
                                                    '<=', 'Arcs')

#continious submodel reformulated help variable definition constraint 1
def FlowStrainMulRouteConstraintRef1ExprRuleLHS_c(model, flow, node_s, node_d):
    return model.FlowStrainMulRoute[flow, node_s, node_d]
AddConstraintContinious('FlowStrainMulRouteConstraint1Ref', FlowStrainMulRouteConstraintRef1ExprRuleLHS_c,
                                                            0, '<=', 'Flows', 'Arcs')

#binary submodel reformulated help variable definition constraint 1
def FlowStrainMulRouteConstraintRef1ExprRuleLHS_b(model, flow, node_s, node_d):
    return -model.Capacity[node_s, node_d] * model.FlowRoute[flow, node_s, node_d]
AddConstraintBinary('FlowStrainMulRouteConstraint1Ref', FlowStrainMulRouteConstraintRef1ExprRuleLHS_b,
                                                        0, '<=', 'Flows', 'Arcs')