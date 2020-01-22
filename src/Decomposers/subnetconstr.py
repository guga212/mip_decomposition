#changed constraints container
sm_constraint_rules_lhs = {}
sm_constraint_rules_rhs = {}


#subnets route constraint
def ConstraintRouteExprRuleLHS(model, flow, subnet, node):
    return 0 \
    + sum(model.FlowRoute[flow, subnet, i, node] for i in model.NodesIn[subnet, node]) \
    - sum(model.FlowRoute[flow, subnet, node, j] for j in model.NodesOut[subnet, node])
def ConstraintRouteExprRuleRHS(model, flow, subnet, node):
    sum_eq = -1 if node == model.Src[flow] else 1 if node == model.Dst[flow] else 0
    return sum_eq
sm_constraint_rules_lhs['RouteConstraint'] = ConstraintRouteExprRuleLHS
sm_constraint_rules_rhs['RouteConstraint'] = ConstraintRouteExprRuleRHS

#subnets nonlinear capacity constraint
def ConstraintCapacityExprRuleLHS(model, subnet, node_s, node_d):
    return sum(model.FlowStrain[flow, subnet] * model.FlowRoute[flow, subnet, node_s, node_d] for flow in model.Flows)
def ConstraintCapacityExprRuleRHS(model, subnet, node_s, node_d):
    return model.Capacity[node_s, node_d]
sm_constraint_rules_lhs['CapacityConstraint'] = ConstraintCapacityExprRuleLHS
sm_constraint_rules_rhs['CapacityConstraint'] = ConstraintCapacityExprRuleRHS    


#subnets help variable definition constraint 1
def FlowStrainMulRouteConstraint1ExprRuleLHS(model, flow, subnet, node_s, node_d):
    return -model.FlowStrainMulRoute[flow, subnet, node_s, node_d] + model.FlowStrain[flow, subnet] \
            +  model.FlowUb[flow] * model.FlowRoute[flow, subnet, node_s, node_d]
def FlowStrainMulRouteConstraint1ExprRuleRHS(model, flow, subnet, node_s, node_d):
    return model.FlowUb[flow]
sm_constraint_rules_lhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleLHS
sm_constraint_rules_rhs['FlowStrainMulRouteConstraint1'] = FlowStrainMulRouteConstraint1ExprRuleRHS

#subnets help variable definition constraint 3
def FlowStrainMulRouteConstraint3ExprRuleLHS(model, flow, subnet, node_s, node_d):
    return model.FlowStrainMulRoute[flow, subnet, node_s, node_d] - model.FlowStrain[flow, subnet]
sm_constraint_rules_lhs['FlowStrainMulRouteConstraint3'] = FlowStrainMulRouteConstraint3ExprRuleLHS
sm_constraint_rules_rhs['FlowStrainMulRouteConstraint3'] = 0

#subnets help variable definition constraint 4
def FlowStrainMulRouteConstraint4ExprRuleLHS(model, flow, subnet, node_s, node_d):
    return model.FlowStrainMulRoute[flow, subnet, node_s, node_d] \
            - model.FlowUb[flow] * model.FlowRoute[flow, subnet, node_s, node_d]
sm_constraint_rules_lhs['FlowStrainMulRouteConstraint4'] = FlowStrainMulRouteConstraint4ExprRuleLHS
sm_constraint_rules_rhs['FlowStrainMulRouteConstraint4'] = 0

#subnets linear capacity constraint
def ConstraintCapacityLinearExprRuleLHS(model, subnet, node_s, node_d):
    return sum(model.FlowStrainMulRoute[flow, subnet, node_s, node_d] for flow in model.Flows )
def ConstraintCapacityLinearExprRuleRHS(model, subnet, node_s, node_d):
    return model.Capacity[node_s, node_d]
sm_constraint_rules_lhs['CapacityConstraintLinear'] = ConstraintCapacityLinearExprRuleLHS
sm_constraint_rules_rhs['CapacityConstraintLinear'] = ConstraintCapacityLinearExprRuleRHS    

#subnets continious variables route constraint (reformulated model)
def ConstraintRouteContiniousExprRuleLHS(model, flow, subnet, node):
    return 0 \
    + sum(model.FlowStrainMulRoute[flow, subnet, i, node] for i in model.NodesIn[subnet, node]) \
    - sum(model.FlowStrainMulRoute[flow, subnet, node, j] for j in model.NodesOut[subnet, node])            
def ConstraintRouteContiniousExprRuleRHS(model, flow, subnet, node):
    sum_eq = -model.FlowStrain[flow, subnet] if node == model.Src[flow] else model.FlowStrain[flow, subnet] if node == model.Dst[flow] else 0
    return sum_eq
sm_constraint_rules_lhs['RouteConstraintContiniousRef'] = ConstraintRouteContiniousExprRuleLHS
sm_constraint_rules_rhs['RouteConstraintContiniousRef'] = ConstraintRouteContiniousExprRuleRHS

#subnets single flow constraint (reformulated model)
def SingleFlowConstraintRuleExprLHS(model, flow, subnet, node):
    return sum(model.FlowRoute[flow, subnet, node, i] for i in model.NodesOut[subnet, node])
sm_constraint_rules_lhs['SingleFlowConstraintRef'] = SingleFlowConstraintRuleExprLHS
sm_constraint_rules_rhs['SingleFlowConstraintRef'] = 1

#subnets help variable definition constraint (reformulated model)
def FlowStrainMulRouteConstraint1RefExprRuleLHS(model, flow, subnet, node_s, node_d):
    return model.FlowStrainMulRoute[flow, subnet, node_s, node_d] \
    - model.FlowUb[flow] * model.FlowRoute[flow, subnet, node_s, node_d]
sm_constraint_rules_lhs['FlowStrainMulRouteConstraint1Ref'] = FlowStrainMulRouteConstraint1RefExprRuleLHS
sm_constraint_rules_rhs['FlowStrainMulRouteConstraint1Ref'] = 0

#subnets linera cpacity constraint (reformulated model)
def ConstraintCapacityRefExprRuleLHS(model, subnet, node_s, node_d):
    return sum(model.FlowStrainMulRoute[flow, subnet, node_s, node_d] for flow in model.Flows )
def ConstraintCapacityRefExprRuleRHS(model, subnet, node_s, node_d):
    return model.Capacity[node_s, node_d]
sm_constraint_rules_lhs['CapacityConstraintRef'] = ConstraintCapacityRefExprRuleLHS
sm_constraint_rules_rhs['CapacityConstraintRef'] = ConstraintCapacityRefExprRuleRHS  