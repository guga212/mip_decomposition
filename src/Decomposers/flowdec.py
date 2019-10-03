import pyomo.environ as pyo
import copy as cp
from .relax import RelaxConstraints

from .gendec import GeneralDecomposer

class FlowDecomposer(GeneralDecomposer):
    def __init__(self, rs_model, coordinator, binding_constraint_name = 'CapacityConstraint'):
        """
        Specialization of the general decomposer.
        Relaxes capacity constraints. Decomposes
        model via flows set.
        """

        relaxed_constraint_name = binding_constraint_name
        relaxed_constraint_range = [ arc for arc in  rs_model.init_data[None]['Arcs'][None] ]        
        decompose_group_init_data = []
        for flow in rs_model.init_data[None]['Flows'][None]:
            init_data_local = cp.deepcopy(rs_model.init_data)
            init_data_local[None]['Flows'][None] = [flow]
            init_data_local[None]['Src'] = { k: v for k, v in init_data_local[None]['Src'].items() if k == flow}
            init_data_local[None]['Dst'] = { k: v for k, v in init_data_local[None]['Dst'].items() if k == flow}
            decompose_group_init_data.append(init_data_local)
        super().__init__(rs_model, [(relaxed_constraint_name, relaxed_constraint_range)], decompose_group_init_data, coordinator)