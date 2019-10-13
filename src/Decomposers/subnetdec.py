import copy as cp
import pyomo.environ as pyo
import ModelGenerator as mg
from .relax import RelaxConstraints
from .gendec import GeneralDecomposer

class ASubNetRoutingStrainModel(mg.ARoutingStrainModel):
    def InitializeBaseSets(self):
        self.Subnets = pyo.Set()
        self.Flows = pyo.Set()
        self.SubnetsNodes = pyo.Set(self.Subnets)
        self.SubnetsArcs = pyo.Set(self.Subnets, dimen = 2)

    def InitializeDerivedSets(self):

        def InitNodes(model):
            ret_val = []
            for subnet in model.SubnetsNodes:
                for subnet_node in model.SubnetsNodes[subnet]:
                    ret_val.append( (subnet, subnet_node) )
            return ret_val
        self.Nodes = pyo.Set(dimen = 2, initialize = InitNodes)

        def InitArcs(model):
            ret_val = []
            for subnet in model.SubnetsArcs:
                for subnet_arc in model.SubnetsArcs[subnet]:
                    ret_val.append( (subnet, *subnet_arc) )
            return ret_val
        self.Arcs= pyo.Set(dimen = 3, initialize = InitArcs)

        def InitRealArcs(model):
            ret_val = []
            for subnet in model.SubnetsArcs:
                for subnet_arc in model.SubnetsArcs[subnet]:
                    if subnet_arc not in ret_val:
                        ret_val.append(subnet_arc)
            return ret_val
        self.RealArcs = pyo.Set(dimen = 2, initialize = InitRealArcs)

        def InitDuplicateddArcs(model):
            arcs_subnets = {}
            for subnet in model.SubnetsArcs:
                for subnet_arc in model.SubnetsArcs[subnet]:
                    if subnet_arc not in arcs_subnets:
                        arcs_subnets[subnet_arc] = (subnet,)
                    else:
                        arcs_subnets[subnet_arc] = ( *arcs_subnets[subnet_arc], subnet )
            ret_value = [ (*subnets, *arc)  for arc, subnets in arcs_subnets.items() if len(subnets) == 2 ]
            return ret_value
        self.DuplicatedArcs = pyo.Set(dimen = 4, initialize = InitDuplicateddArcs)

        def NodesOut_init(model, node_subnet, node):
            retval = []
            for (i,j) in model.SubnetsArcs[node_subnet]:
                if i == node:
                    retval.append(j)
            return retval
        self.NodesOut = pyo.Set(self.Nodes, initialize = NodesOut_init)
        def NodesIn_init(model, node_subnet, node):
            retval = []
            for (i,j) in model.SubnetsArcs[node_subnet]:
                if j == node:
                    retval.append(i)
            return retval
        self.NodesIn = pyo.Set(self.Nodes, initialize = NodesIn_init)

    def InitializeParameters(self):
        self.Src = pyo.Param(self.Flows, within = pyo.NonNegativeIntegers)
        self.Dst = pyo.Param(self.Flows, within = pyo.NonNegativeIntegers)       
        self.Capacity = pyo.Param(self.RealArcs, within = pyo.NonNegativeReals)
        self.FlowLb = pyo.Param(within=pyo.NonNegativeIntegers, mutable = True)
        self.FlowUb = pyo.Param(within=pyo.NonNegativeIntegers, mutable = True)        
        def StrainBoundsRule(model, *args):
            return ( model.FlowLb, model.FlowUb )
        self.StrainBoundsRule = StrainBoundsRule

    def InitializeVariables(self):
        self.FlowStrain = pyo.Var(self.Flows, self.Subnets, domain = pyo.NonNegativeReals, bounds = self.StrainBoundsRule )
        self.FlowRoute = pyo.Var(self.Flows, self.Arcs, domain = pyo.Binary)
    def __init__(self):
        super().__init__()

#changed constraints container
sm_constraint_rules_lhs = {}
sm_constraint_rules_rhs = {}
M_MULT = 1.1

#subnets route constraint
def ConstraintRouteExprRuleLHS(model, flow, subnet, node):
    return 0 \
    + sum(model.FlowRoute[flow, subnet, i, node] for i in model.NodesIn[subnet, node]) \
    - sum(model.FlowRoute[flow, subnet, node, j] for j in model.NodesOut[subnet, node])
def ConstraintRouteExprRuleRHS(model, flow, subnet, node):
    sum_eq = -1 if node == model.Src[flow] else 1 if node == model.Dst[flow] else 0
    return sum_eq
sm_constraint_rules_lhs['RouteConstraint'] = ConstraintRouteExprRuleLHS
sm_constraint_rules_rhs['RouteConstraint'] = ConstraintRouteExprRuleLHS

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
            + M_MULT * model.FlowUb * model.FlowRoute[flow, subnet, node_s, node_d]
def FlowStrainMulRouteConstraint1ExprRuleRHS(model, flow, subnet, node_s, node_d):
    return M_MULT * model.FlowUb
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
            - M_MULT * model.FlowUb * model.FlowRoute[flow, subnet, node_s, node_d]
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
    - model.Capacity[node_s, node_d] * model.FlowRoute[flow, subnet, node_s, node_d]
sm_constraint_rules_lhs['FlowStrainMulRouteConstraint1Ref'] = FlowStrainMulRouteConstraint1RefExprRuleLHS
sm_constraint_rules_rhs['FlowStrainMulRouteConstraint1Ref'] = 0

#subnets linera cpacity constraint (reformulated model)
def ConstraintCapacityRefExprRuleLHS(model, subnet, node_s, node_d):
    return sum(model.FlowStrainMulRoute[flow, subnet, node_s, node_d] for flow in model.Flows )
def ConstraintCapacityRefExprRuleRHS(model, subnet, node_s, node_d):
    return model.Capacity[node_s, node_d]
sm_constraint_rules_lhs['CapacityConstraintRef'] = ConstraintCapacityRefExprRuleLHS
sm_constraint_rules_rhs['CapacityConstraintRef'] = ConstraintCapacityRefExprRuleRHS    


class SubnetsDecomposer(GeneralDecomposer):
    def __init__(self, rs_model, coordinator, subnets_nodes):
        """
        Specialization of the general decomposer.
        Add addditional constraints and modifie
        objective function. Decomposes
        model via given subnets.
        """

        #copy given model to modify it safely
        rs_model_sm = cp.deepcopy(rs_model)
        amodel_working = rs_model_sm.amodel

        #create abstract subnet decomosition model
        amodel_sm = ASubNetRoutingStrainModel()

        #add objective from the original model
        obj_ref_tmp = amodel_working.Obj
        amodel_working.del_component(amodel_working.Obj)
        amodel_sm.Obj = obj_ref_tmp

        #add help varaible if presented in original model
        if hasattr(amodel_working, 'FlowStrainMulRoute') == True:
            amodel_sm.FlowStrainMulRoute = pyo.Var(amodel_sm.FlowRoute.index_set(), domain = pyo.NonNegativeReals)

        #add constraints presented in original model
        for constraint in amodel_working.component_objects(pyo.Constraint, active = True):
            lhs = sm_constraint_rules_lhs[constraint.name]
            rhs = sm_constraint_rules_rhs[constraint.name]
            cname = amodel_working.Suffix[constraint]['CompareName']
            sets_names = amodel_working.Suffix[constraint]['SetsNames']
            mg.constrmk.AddConstraint(amodel_sm, constraint.name, lhs, 
                                        rhs, cname, *sets_names)

        #equality constraints for the replicated flows
        def ConstraintFlowStrainEqLhsRule(model, flow, subnet):
            subnet_tab = [subnet for subnet in model.Subnets]
            cur_indx = subnet_tab.index(subnet)
            next_indx = (cur_indx + 1) % len(subnet_tab)
            return model.FlowStrain[flow, subnet_tab[cur_indx]] \
                    - model.FlowStrain[flow, subnet_tab[next_indx]]
        mg.constrmk.AddConstraint(amodel_sm, "FlowStrainEqConstraint", ConstraintFlowStrainEqLhsRule, 
                                    0, '==', 'Flows', 'Subnets')

        #equality constraints for the duplicated routes
        def ConstraintInterRouteEqLhsRule(model, flow, subnet_1, subnet_2, node_s, node_d):
            return model.FlowRoute[flow, subnet_1, node_s, node_d] \
                    - model.FlowRoute[flow, subnet_2, node_s, node_d]
        mg.constrmk.AddConstraint(amodel_sm, "InterRouteEqConstraint", ConstraintInterRouteEqLhsRule,
                                    0, '==', 'Flows', 'DuplicatedArcs')

        #modify init data for the subnet model
        init_data_sm = rs_model_sm.init_data
        init_data_sm[None]['Subnets'] = { None : [] }
        init_data_sm[None]['SubnetsNodes'] = {}
        init_data_sm[None]['SubnetsArcs'] = {}
        for subnet, nodes in subnets_nodes.items():
            init_data_sm[None]['Subnets'][None].append(subnet)
            init_data_sm[None]['SubnetsNodes'][subnet] = nodes
            init_data_sm[None]['SubnetsArcs'][subnet] = []
            for arc in init_data_sm[None]['Arcs'][None]:
                if arc[0] in nodes or arc[1] in nodes:
                    init_data_sm[None]['SubnetsArcs'][subnet].append(arc)
        del init_data_sm[None]['Nodes']
        del init_data_sm[None]['Arcs']
        
        #create concrete model
        cmodel_sm = amodel_sm.create_instance(data = init_data_sm)

        #inittialize rs model structure
        rs_model_sm.amodel = amodel_sm
        rs_model_sm.init_data = init_data_sm
        rs_model_sm.cmodel = cmodel_sm

        #create relaxation require data
        relaxed_constraint_name_1 = "FlowStrainEqConstraint"
        def RelaxedSet1InitializeGenerator():
            def RelaxedSetInitialize(model):
                ret_val = [ (flow, subnet) for flow in model.Flows for subnet in model.Subnets]
                return ret_val
            return RelaxedSetInitialize
        # relaxed_constraint_range_1 = [ (flow, subnet) for flow in rs_model_sm.init_data[None]['Flows'][None] 
        #                                 for subnet in rs_model_sm.init_data[None]['Subnets'][None] ]
        relaxed_constraint_name_2 = "InterRouteEqConstraint"
        def RelaxedSet2InitializeGenerator():
            def RelaxedSetInitialize(model):
                ret_val = [ (flow, *arc) for flow in model.Flows for arc in model.DuplicatedArcs]
                return ret_val
            return RelaxedSetInitialize
        # relaxed_constraint_range_2 = [ (flow, *arc) for flow in rs_model_sm.init_data[None]['Flows'][None] 
        #                                 for arc in rs_model_sm.cmodel.DuplicatedArcs]
        relaxed_data = [ (relaxed_constraint_name_1, RelaxedSet1InitializeGenerator()), 
                        (relaxed_constraint_name_2, RelaxedSet2InitializeGenerator()) ] 

        #create decomposition required data
        decompose_group_init_data = []
        for subnet in rs_model_sm.init_data[None]['Subnets'][None]:
            init_data_local = cp.deepcopy(rs_model_sm.init_data)
            init_data_local[None]['Subnets'][None] = [subnet]
            init_data_local[None]['SubnetsNodes'] = { subnet: rs_model_sm.init_data[None]['SubnetsNodes'][subnet] }
            init_data_local[None]['SubnetsArcs'] = { subnet: rs_model_sm.init_data[None]['SubnetsArcs'][subnet] }
            init_data_local[None]['Capacity'] = { arc: rs_model_sm.init_data[None]['Capacity'][arc] 
                                                    for arc in rs_model_sm.init_data[None]['Capacity'] 
                                                    if arc in rs_model_sm.init_data[None]['SubnetsArcs'][subnet] 
                                                    }
            decompose_group_init_data.append(init_data_local)
        super().__init__(rs_model_sm, relaxed_data, decompose_group_init_data, coordinator)

        # import SolverManager as sm
        # glpk_solver = sm.milpsolvers.GlpkSolver()
        # res = glpk_solver.Solve(cmodel_sm)

        debug_val = 1
        