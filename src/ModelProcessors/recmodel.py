import ModelGenerator as mg
import pyomo.environ as pyo
import copy as cp

def RecoverFeasibleStrain(rs_model, routes, solver):
    """Recover optimal and feasible solution for the given routes"""

    #update route for required format
    routes_updated = {}
    for indx, route in enumerate(routes):
        ru = { (indx, *k) : v  for k, v in route.items() }
        routes_updated.update(ru)

    #copy model for less interference
    amodel = cp.deepcopy(rs_model.amodel)

    #create basic model
    rec_amodel = mg.RsModelGenerator(mg.RouteConstraintsGenerator(), mg.NonlinearCapacityConstraintsGenerator()).CreateAbstractModel()
    
    #update objectve
    obj = amodel.Obj
    amodel.del_component(obj)
    rec_amodel.Obj = obj

    #create concrete instance
    rec_cmodel = rec_amodel.create_instance(data = rs_model.init_data)
    
    #fix variables
    for indx in routes_updated:
        rec_cmodel.FlowRoute[indx].fix(routes_updated[indx])

    #solve recovery model
    ret_val = solver.Solve(rec_cmodel)

    #add generated model to the output
    if ret_val is not None:
        ret_val['Cmodel'] = rec_cmodel

    return ret_val
    