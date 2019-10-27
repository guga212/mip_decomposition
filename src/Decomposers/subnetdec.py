import copy as cp
import pyomo.environ as pyo
import ModelGenerator as mg
from ModelGenerator.mgenerator import RsModel
from .subnetconstr import sm_constraint_rules_lhs, sm_constraint_rules_rhs
from .subnetobj import ObjectiveLin, ObjectiveQuad, ObjectiveLog
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

        def InitOriginalArcs(model):
            duplicated_arcs = [ (indx[1], indx[2], indx[3]) for indx in model.DuplicatedArcs ]
            original_arcs = [ arc for arc in model.Arcs if arc not in duplicated_arcs ]
            return original_arcs
        self.OriginalArcs = pyo.Set(dimen = 3, initialize = InitOriginalArcs)                

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
        self.FlowStrain = pyo.Var(self.Flows, self.Subnets, domain = pyo.PositiveReals, bounds = self.StrainBoundsRule )
        self.FlowRoute = pyo.Var(self.Flows, self.Arcs, domain = pyo.Binary)
    
    def __init__(self):
        super().__init__()
        self.name = 'SubnetRoutingStrainAbstractModel'

  

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
        amodel_sm.FlowStrainWeight = pyo.Param(mutable = True, default = amodel_working.FlowStrainWeight.default() / len(subnets_nodes))
        amodel_sm.FlowRouteWeight = pyo.Param(mutable = True, default = amodel_working.FlowRouteWeight.default())
        obj_ref_tmp = amodel_working.Obj
        objective_name = amodel_working.Suffix[amodel_working.Obj]
        if objective_name == 'Linear':
            amodel_working.del_component(amodel_working.Obj)
            amodel_sm.Obj = obj_ref_tmp
            amodel_sm.Obj.rule = ObjectiveLin
            amodel_sm.Suffix[amodel_sm.Obj] = 'Linear'
        if objective_name == 'Quadratic':
            amodel_working.del_component(amodel_working.Obj)
            amodel_sm.Obj = obj_ref_tmp
            amodel_sm.Obj.rule = ObjectiveQuad
            amodel_sm.Suffix[amodel_sm.Obj] = 'Quadratic'
        if objective_name == 'Logarithmic':
            amodel_working.del_component(amodel_working.Obj)
            amodel_sm.Obj = obj_ref_tmp
            amodel_sm.Obj.rule = ObjectiveLog
            amodel_sm.Suffix[amodel_sm.Obj] = 'Logarithmic'

        #add help varaible if presented in original model
        if hasattr(amodel_working, 'FlowStrainMulRoute') == True:
            amodel_sm.FlowStrainMulRoute = pyo.Var(amodel_sm.FlowRoute.index_set(), domain = pyo.PositiveReals)

        #add constraints presented in original model
        for constraint in amodel_working.component_objects(pyo.Constraint, active = True):
            lhs = sm_constraint_rules_lhs[constraint.name]
            rhs = sm_constraint_rules_rhs[constraint.name]
            cname = amodel_working.Suffix[constraint]['CompareName']
            sets_names = amodel_working.Suffix[constraint]['SetsNames']
            mg.constrmk.AddConstraint(amodel_sm, constraint.name, lhs, 
                                        rhs, cname, *sets_names)

        #equality constraints for the replicated flows
        def ConstraintFlowStrainEqLhsRuleGenerator(subnet_tab):

            def ConstraintFlowStrainEqLhsRule(model, flow, subnet):
                cur_indx = subnet_tab.index(subnet)
                next_indx = (cur_indx + 1) % len(subnet_tab)
                indx_1 = (flow, subnet_tab[cur_indx])
                if indx_1 in model.FlowStrain.index_set():
                    strain_1 = model.FlowStrain[indx_1]
                else:
                    strain_1 = 0
                indx_2 = (flow, subnet_tab[next_indx])
                if indx_2 in model.FlowStrain.index_set():
                    strain_2 = model.FlowStrain[indx_2]
                else:
                    strain_2 = 0
                return strain_1 - strain_2

            return ConstraintFlowStrainEqLhsRule
        subnet_tab = [ subnet for subnet in subnets_nodes ]
        mg.constrmk.AddConstraint(amodel_sm, "FlowStrainEqConstraint", ConstraintFlowStrainEqLhsRuleGenerator(subnet_tab), 
                                    0, '==', 'Flows', 'Subnets')

        #equality constraints for the duplicated routes
        def ConstraintInterRouteEqLhsRule(model, flow, subnet_1, subnet_2, node_s, node_d):
            indx_1 = flow, subnet_1, node_s, node_d
            if indx_1 in model.FlowRoute.index_set():
                route_1 = model.FlowRoute[indx_1]
            else:
                route_1 = 0
            indx_2 = flow, subnet_2, node_s, node_d
            if indx_2 in model.FlowRoute.index_set():
                route_2 = model.FlowRoute[indx_2]
            else:
                route_2 = 0
            return route_1 - route_2            
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
        relaxed_constraint_range_1 = [ (flow, subnet) for flow in cmodel_sm.Flows for subnet in cmodel_sm.Subnets]
        relaxed_constraint_name_2 = "InterRouteEqConstraint"
        relaxed_constraint_range_2 = [ (flow, *arc) for flow in cmodel_sm.Flows for arc in cmodel_sm.DuplicatedArcs]
        relaxed_data = [ (relaxed_constraint_name_1, relaxed_constraint_range_1), 
                (relaxed_constraint_name_2, relaxed_constraint_range_2) ]

        #create decomposition required data
        decompose_group_local_rs_models = []
        for subnet in rs_model_sm.init_data[None]['Subnets'][None]:
            lrsm = RsModel()
            lrsm.amodel = cp.deepcopy(rs_model_sm.amodel)
            lrsm.cmodel = None
            init_data_local = cp.deepcopy(rs_model_sm.init_data)
            init_data_local[None]['Subnets'][None] = [subnet]
            init_data_local[None]['SubnetsNodes'] = { subnet: rs_model_sm.init_data[None]['SubnetsNodes'][subnet] }
            init_data_local[None]['SubnetsArcs'] = { subnet: rs_model_sm.init_data[None]['SubnetsArcs'][subnet] }
            init_data_local[None]['Capacity'] = { arc: rs_model_sm.init_data[None]['Capacity'][arc] 
                                                    for arc in rs_model_sm.init_data[None]['Capacity'] 
                                                    if arc in rs_model_sm.init_data[None]['SubnetsArcs'][subnet] 
                                                    }
            lrsm.init_data = init_data_local
            decompose_group_local_rs_models.append(lrsm)
        super().__init__(rs_model_sm, relaxed_data, decompose_group_local_rs_models, coordinator)