from .amodel import ARoutingStrainModel

class RsModelGenerator:
    def __init__(self, *args):
        self.amodel = ARoutingStrainModel()
        for arg in args:
            arg(self.amodel.model)
    def CreateInstance(self, flows, src_dst, nodes, arcs, capacity, flw_bounds):

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

        return self.amodel.model.create_instance(data = init_data)
