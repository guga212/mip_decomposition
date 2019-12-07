import ModelGenerator as mg
import pyomo.environ as pyo
import copy as cp

from .sepvarconstr import sv_continious_constraint_rules_lhs, sv_continious_constraint_rules_rhs, \
                            sv_binary_constraint_rules_lhs, sv_binary_constraint_rules_rhs 
from .sepvarobj import ObjectiveContLin, ObjectiveContQuad, ObjectiveContLog, ObjectiveBin
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


class SepVarDecomposer(GeneralDecomposer):
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

        mg.constrmk.AddConstraint(rs_model_cont.amodel, 'FlowStrainMulRouteConstraint1',
                                    sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint1'],
                                    sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint1'],
                                    '<=', 'Flows', 'Arcs')
        mg.constrmk.AddConstraint(rs_model_cont.amodel, 'FlowStrainMulRouteConstraint3',
                                    sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint3'],
                                    sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint3'],
                                    '<=', 'Flows', 'Arcs')
        mg.constrmk.AddConstraint(rs_model_cont.amodel, 'FlowStrainMulRouteConstraint4',
                                    sv_continious_constraint_rules_lhs['FlowStrainMulRouteConstraint4'],
                                    sv_continious_constraint_rules_rhs['FlowStrainMulRouteConstraint4'],
                                    '<=', 'Flows', 'Arcs')
        mg.constrmk.AddConstraint(rs_model_cont.amodel, 'CapacityConstraintLinear', 
                                    sv_continious_constraint_rules_lhs['CapacityConstraintLinear'],
                                    sv_continious_constraint_rules_rhs['CapacityConstraintLinear'],
                                    '<=', 'Arcs')
        
        rs_model_cont.amodel.FlowStrainWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowStrainWeight.default())
        rs_model_cont.amodel.FlowRouteWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowRouteWeight.default())
        objective_name = rs_model_sv.amodel.Suffix[rs_model_sv.amodel.Obj]
        if objective_name == 'Linear':
            rs_model_cont.amodel.Obj = pyo.Objective(rule = ObjectiveContLin, sense = pyo.minimize)
            rs_model_cont.amodel.Suffix[rs_model_cont.amodel.Obj] = 'Linear'
        if objective_name == 'Quadratic':
            rs_model_cont.amodel.Obj = pyo.Objective(rule = ObjectiveContQuad, sense = pyo.minimize)
            rs_model_cont.amodel.Suffix[rs_model_cont.amodel.Obj] = 'Quadratic'
        if objective_name == 'Logarithmic':
            rs_model_cont.amodel.Obj = pyo.Objective(rule = ObjectiveContLog, sense = pyo.minimize)
            rs_model_cont.amodel.Suffix[rs_model_cont.amodel.Obj] = 'Logarithmic'

        rs_model_cont.init_data = cp.deepcopy(rs_model_sv.init_data)
        rs_model_cont.cmodel = None

        #binary local model
        rs_model_bin = mg.mgenerator.RsModel()
        rs_model_bin.amodel = ASepVarBinary()
        mg.constrmk.AddConstraint(rs_model_bin.amodel, 'RouteConstraint',
                            sv_binary_constraint_rules_lhs['RouteConstraint'],
                            sv_binary_constraint_rules_rhs['RouteConstraint'],
                            '==', 'Flows', 'Nodes')
        mg.constrmk.AddConstraint(rs_model_bin.amodel, 'FlowStrainMulRouteConstraint1',
                                    sv_binary_constraint_rules_lhs['FlowStrainMulRouteConstraint1'],
                                    sv_binary_constraint_rules_rhs['FlowStrainMulRouteConstraint1'],
                                    '<=', 'Flows', 'Arcs')
        mg.constrmk.AddConstraint(rs_model_bin.amodel, 'FlowStrainMulRouteConstraint4',
                                    sv_binary_constraint_rules_lhs['FlowStrainMulRouteConstraint4'],
                                    sv_binary_constraint_rules_rhs['FlowStrainMulRouteConstraint4'],
                                    '<=', 'Flows', 'Arcs')

        rs_model_bin.amodel.FlowStrainWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowStrainWeight.default())
        rs_model_bin.amodel.FlowRouteWeight = pyo.Param(mutable = True, default = rs_model_sv.amodel.FlowRouteWeight.default())
        objective_name = rs_model_sv.amodel.Suffix[rs_model_sv.amodel.Obj]
        rs_model_bin.amodel.Obj = pyo.Objective(rule = ObjectiveBin, sense = pyo.minimize)
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

        super().__init__(rs_model_sv, relaxation_data, decompose_group_local_rs_model, coordinator)