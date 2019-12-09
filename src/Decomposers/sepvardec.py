import ModelGenerator as mg
import pyomo.environ as pyo
import copy as cp

from .sepvarconstr import sv_continious_constraint_rules, sv_binary_constraint_rules
from .sepvarobj import sv_objectives
from .gendec import GeneralDecomposer

class ASepVarContinious(mg.ARoutingStrainModel):
    def InitializeVariables(self):
        self.FlowStrain = pyo.Var(self.Flows, domain = pyo.PositiveReals, bounds = self.StrainBoundsRule )
        self.FlowStrainMulRoute = pyo.Var(self.Flows, self.Arcs, domain = pyo.PositiveReals)
    def __init__(self):
        super().__init__()
        self.name = 'SeparateVariableContiniousModel'
class ASepVarBinary(mg.ARoutingStrainModel):
    def InitializeVariables(self):
        def RouteBoundsRule(model, *args):
            return (0, 1)
        self.FlowRoute = pyo.Var(self.Flows, self.Arcs, domain = pyo.NonNegativeReals, bounds = RouteBoundsRule)
    def __init__(self):
        super().__init__()
        self.name = 'SeparateVariableBinaryModel'

class SepVarDecomposerOrig(GeneralDecomposer):
    def __init__(self, rs_model, coordinator):
        """
        Specialization of the general decomposer.
        Relaxes constraints coupling continious
        binary variables.
        """

        #relax constraints which has continious and binary constraints simultaneously
        relaxed_constraint_name_1 = 'FlowStrainMulRouteConstraint1'
        relaxed_constraint_name_2 = 'FlowStrainMulRouteConstraint4'
        relaxed_constraint_range = [ (flow, *arc) for flow in rs_model.init_data[None]['Flows'][None] 
                                        for arc in rs_model.init_data[None]['Arcs'][None] ]

        #define data used for the relaxation
        relaxation_data = [ (relaxed_constraint_name_1, relaxed_constraint_range),  
                            (relaxed_constraint_name_2, relaxed_constraint_range) ]

        #complete model
        rs_model_sv = cp.deepcopy(rs_model)

        #continious local model
        rs_model_cont = mg.mgenerator.RsModel()
        rs_model_cont.amodel = ASepVarContinious()

        for constraint in rs_model_sv.amodel.component_objects(pyo.Constraint):
            if constraint.name in sv_continious_constraint_rules:
                constraint_data = sv_continious_constraint_rules[constraint.name]
                mg.constrmk.AddConstraint(rs_model_cont.amodel, constraint.name,
                                            constraint_data['lhs'], constraint_data['rhs'],
                                            constraint_data['sign'], *constraint_data['sets'])
        
        rs_model_cont.amodel.FlowStrainWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowStrainWeight.default())
        rs_model_cont.amodel.FlowRouteWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowRouteWeight.default())
        objective_name = rs_model_sv.amodel.Suffix[rs_model_sv.amodel.Obj]        
        rs_model_cont.amodel.Obj = pyo.Objective(rule = sv_objectives[objective_name], sense = pyo.minimize)
        rs_model_cont.amodel.Suffix[rs_model_cont.amodel.Obj] = objective_name

        rs_model_cont.init_data = cp.deepcopy(rs_model_sv.init_data)
        rs_model_cont.cmodel = None


        #binary local model
        rs_model_bin = mg.mgenerator.RsModel()
        rs_model_bin.amodel = ASepVarBinary()
        for constraint in rs_model_sv.amodel.component_objects(pyo.Constraint):
            if constraint.name in sv_binary_constraint_rules:
                constraint_data = sv_binary_constraint_rules[constraint.name]
                mg.constrmk.AddConstraint(rs_model_bin.amodel, constraint.name,
                                            constraint_data['lhs'], constraint_data['rhs'],
                                            constraint_data['sign'], *constraint_data['sets'])

        rs_model_bin.amodel.FlowStrainWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowStrainWeight.default())
        rs_model_bin.amodel.FlowRouteWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowRouteWeight.default())
        objective_name = rs_model_sv.amodel.Suffix[rs_model_sv.amodel.Obj]
        rs_model_bin.amodel.Obj = pyo.Objective(rule = sv_objectives['Route'], sense = pyo.minimize)
        rs_model_bin.amodel.Suffix[rs_model_bin.amodel.Obj] = objective_name
        
        rs_model_bin.init_data = cp.deepcopy(rs_model_sv.init_data)
        rs_model_bin.cmodel = None

        #decompose bin models via flows
        rs_model_bin_decomposed = []
        for flow in rs_model.init_data[None]['Flows'][None]:
            rsmbd = cp.deepcopy(rs_model_bin)
            init_data_local = cp.deepcopy(rs_model_sv.init_data)
            init_data_local[None]['Flows'][None] = [flow]
            init_data_local[None]['Src'] = { k: v for k, v in init_data_local[None]['Src'].items() if k == flow}
            init_data_local[None]['Dst'] = { k: v for k, v in init_data_local[None]['Dst'].items() if k == flow}
            rsmbd.init_data = init_data_local
            rs_model_bin_decomposed.append(rsmbd)

        #decomposed groups initialization
        decompose_group_local_rs_model = [rs_model_cont, *rs_model_bin_decomposed]


        #####DEBUG######
        # rs_model_no_decomposition = cp.deepcopy(rs_model)
        # rs_model_sv = cp.deepcopy(rs_model)
        # decompose_group_local_rs_model = [rs_model_no_decomposition]

        # # relaxed_constraint_name_1 = 'FlowStrainMulRouteConstraint1'
        # # relaxed_constraint_name_2 = 'FlowStrainMulRouteConstraint4'
        # # relaxed_constraint_range = [ (flow, *arc) for flow in rs_model.init_data[None]['Flows'][None] 
        # #                                 for arc in rs_model.init_data[None]['Arcs'][None] ]
        # # relaxation_data = [ (relaxed_constraint_name_1, relaxed_constraint_range),  
        # #                     (relaxed_constraint_name_2, relaxed_constraint_range) ]

        # # relaxed_constraint_name = 'FlowStrainMulRouteConstraint1'
        # # relaxed_constraint_range = [ (flow, *arc) for flow in rs_model.init_data[None]['Flows'][None] 
        # #                                 for arc in rs_model.init_data[None]['Arcs'][None] ]
        # # relaxation_data = [ (relaxed_constraint_name, relaxed_constraint_range)]

        # # relaxed_constraint_name_1 = 'CapacityConstraintLinear'
        # # relaxed_constraint_range = [ arc for arc in rs_model.init_data[None]['Arcs'][None]]
        # # relaxation_data = [ (relaxed_constraint_name_1, relaxed_constraint_range) ]
        ################

        GeneralDecomposer.__init__(self, rs_model_sv, relaxation_data, decompose_group_local_rs_model, coordinator)

class SepVarDecomposerRef(GeneralDecomposer):
    def __init__(self, rs_model, coordinator):
        """
        Specialization of the general decomposer.
        Relaxes constraints coupling continious
        binary variables for the reformulated problem.
        """

        #complete model
        rs_model_sv = cp.deepcopy(rs_model)

        #continious local model
        rs_model_cont = cp.deepcopy(rs_model)
        rs_model_cont.amodel.name = 'SeparateVariableContiniousModel'
        rs_model_cont.amodel.del_component(rs_model_cont.amodel.Obj)
        rs_model_cont.amodel.del_component(rs_model_cont.amodel.FlowRoute)
        rs_model_cont.amodel.del_component(rs_model_cont.amodel.SingleFlowConstraintRef)
        rs_model_cont.amodel.del_component(rs_model_cont.amodel.FlowStrainMulRouteConstraint1Ref)
        rs_model_cont.amodel.del_component(rs_model_cont.amodel.FlowStrainMulRouteConstraint1Ref_index)
        constraint_data = sv_continious_constraint_rules['FlowStrainMulRouteConstraint1Ref']
        mg.constrmk.AddConstraint(rs_model_cont.amodel, 'FlowStrainMulRouteConstraint1Ref',
                                    constraint_data['lhs'], constraint_data['rhs'],
                                    constraint_data['sign'], *constraint_data['sets'])
        objective_name = rs_model_sv.amodel.Suffix[rs_model_sv.amodel.Obj]        
        rs_model_cont.amodel.Obj = pyo.Objective(rule = sv_objectives[objective_name], sense = pyo.minimize)
        rs_model_cont.amodel.Suffix[rs_model_cont.amodel.Obj] = objective_name
        rs_model_cont.cmodel = None

        #binary local model
        rs_model_bin = cp.deepcopy(rs_model)
        rs_model_bin.amodel.name = 'SeparateVariableBinaryModel'
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.Obj)
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.FlowStrain)
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.FlowStrainMulRoute)
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.CapacityConstraintRef)
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.RouteConstraintContiniousRef)
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.FlowStrainMulRouteConstraint1Ref)
        rs_model_bin.amodel.del_component(rs_model_bin.amodel.FlowStrainMulRouteConstraint1Ref_index)
        constraint_data = sv_binary_constraint_rules['FlowStrainMulRouteConstraint1Ref']
        mg.constrmk.AddConstraint(rs_model_bin.amodel, 'FlowStrainMulRouteConstraint1Ref',
                                    constraint_data['lhs'], constraint_data['rhs'],
                                    constraint_data['sign'], *constraint_data['sets'])
        rs_model_bin.amodel.Obj = pyo.Objective(rule = sv_objectives['Route'], sense = pyo.minimize)
        rs_model_bin.cmodel = None
        
        #decomposed groups initialization
        decompose_group_local_rs_model = [rs_model_cont, rs_model_bin]

        #relax constraints which has continious and binary constraints simultaneously
        relaxed_constraint_name = 'FlowStrainMulRouteConstraint1Ref'
        relaxed_constraint_range = [ (flow, *arc) for flow in rs_model.init_data[None]['Flows'][None] 
                                        for arc in rs_model.init_data[None]['Arcs'][None] ]
        #define data used for the relaxation
        relaxation_data = [(relaxed_constraint_name, relaxed_constraint_range)]

        ###DEBUG###
        #decompose_group_local_rs_model = [ cp.deepcopy(rs_model) ]
        ###########

        GeneralDecomposer.__init__(self, rs_model_sv, relaxation_data, decompose_group_local_rs_model, coordinator)

class SepVarDecomposer(SepVarDecomposerOrig, SepVarDecomposerRef):
    def __init__(self, rs_model, coordinator):
        if  hasattr(rs_model.amodel, 'Reformulated'):
            SepVarDecomposerRef.__init__(self, rs_model, coordinator)
        else:
            SepVarDecomposerOrig.__init__(self, rs_model, coordinator)

