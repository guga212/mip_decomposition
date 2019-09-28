import copy as cp
from .amodel import ARoutingStrainModel

class RsModel:
    def __init__(self):
        self.amodel = None
        self.init_data = None
        self.cmodel = None


class RsModelGenerator:
    def __init__(self, *args):
        self.amodel = ARoutingStrainModel()
        for arg in args:
            arg(self.amodel)

    def CreateAbstractModel(self):
        return cp.deepcopy(self.amodel)

    def CreateInitData(self, flows, src_dst, nodes, arcs, capacity, flw_bounds):
        init_data = {None: {
                'Flows'    : {None : [ indx for indx, val in enumerate(flows) ]},
                'FlowLb'   : {None : flw_bounds[0]},
                'FlowUb'   : {None : flw_bounds[1]},
                'Src'      : {k : v[0] for k, v in src_dst.items()},
                'Dst'      : {k : v[1] for k, v in src_dst.items()},
                'Nodes'    : {None : nodes},
                'Arcs'     : {None : arcs},
                'Capacity' : capacity,
            }      }
        return init_data

    def CreateConcreteModel(self, init_data):
        return self.amodel.create_instance(data = init_data)

    def CreateCompletRsModel(self, flows, src_dst, nodes, arcs, capacity, flw_bounds):
        rs_model = RsModel()
        rs_model.amodel = self.CreateAbstractModel()
        rs_model.init_data = self.CreateInitData(flows, src_dst, nodes, arcs, capacity, flw_bounds)
        rs_model.cmodel = self.amodel.create_instance(data = rs_model.init_data)
        return rs_model
