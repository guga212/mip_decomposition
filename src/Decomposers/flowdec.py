import pyomo.environ as pyo
import copy as cp

from ModelGenerator.mgenerator import RsModel
from .gendec import GeneralDecomposer

class FlowDecomposer(GeneralDecomposer):
    def __init__(self, rs_model, coordinator):
        """
        Specialization of the general decomposer.
        Relaxes capacity constraints. Decomposes
        model via flows set.
        """

        if hasattr(rs_model.amodel, 'CapacityConstraint'):
            relaxed_constraint_name = 'CapacityConstraint'

        if hasattr(rs_model.amodel, 'CapacityConstraintLinear'):
            relaxed_constraint_name = 'CapacityConstraintLinear'

        relaxed_constraint_range = [ arc for arc in  rs_model.init_data[None]['Arcs'][None] ]
        relaxation_data = [(relaxed_constraint_name, relaxed_constraint_range)]

        decompose_local_rs_models = []
        for flow in rs_model.init_data[None]['Flows'][None]:
            lrsm = RsModel()
            lrsm.amodel = cp.deepcopy(rs_model.amodel)
            init_data_local = cp.deepcopy(rs_model.init_data)
            init_data_local[None]['Flows'][None] = [flow]
            init_data_local[None]['Src'] = { k: v for k, v in init_data_local[None]['Src'].items() if k == flow}
            init_data_local[None]['Dst'] = { k: v for k, v in init_data_local[None]['Dst'].items() if k == flow}
            lrsm.init_data = init_data_local
            lrsm.cmodel = None
            decompose_local_rs_models.append(lrsm)
        super().__init__(rs_model, relaxation_data, decompose_local_rs_models, coordinator)