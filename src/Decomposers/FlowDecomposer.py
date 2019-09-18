import copy as cp
import pyomo.environ as pyo

class FlowDecomposition:
    def __init__(self, model):
        #abstract local model
        self.amodel_local = cp.deepcopy(model.amodel)
        self.amodel_local.LagrangeMult = pyo.Param(self.amodel_local.Arcs, mutable = True, within = pyo.NonNegativeReals)
        
        #create concrete local models
        self.cmodels_local = []
        flows = model.init_data[None]['Flows'][None]
        for flow_indx, flow in enumerate(flows):
            init_data_local = cp.deepcopy(model.init_data)
            init_data_local[None]['Flows'][None] = [ init_data_local[None]['Flows'][None][flow_indx] ]
            init_data_local[None]['Src'] = { k: v for k, v in init_data_local[None]['Src'].items() if k == flow}
            init_data_local[None]['Dst'] = { k: v for k, v in init_data_local[None]['Dst'].items() if k == flow}
            self.cmodels_local.append(self.amodel_local.create_instance(data = init_data_local))
        debug_val = 1

        
    def Solve(self):
        pass
    