from .amodel import ARoutingStrainModel

class RsModelGenerator:
    def __init__(self, var_mk = None, obj_mk = None, constr_mk = None):
        self.amodel = ARoutingStrainModel()
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
