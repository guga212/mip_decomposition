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
            arg(self.amodel.model)
    def CreateModel(self, flows, src_dst, nodes, arcs, capacity, flw_bounds):

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

        rs_model = RsModel()
        rs_model.amodel = self.amodel.model
        rs_model.init_data = init_data
        rs_model.cmodel = self.amodel.model.create_instance(data = init_data)

        return rs_model
