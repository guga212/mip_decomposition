import ModelGenerator as mg
import SolverManager as sm
import pyomo.environ as pyo
import copy as cp
from collections import defaultdict
from .recmodel import RecoverFeasibleStrain


def EstimateObjectvie(rs_model, preserve_feasibility = True):
    """Find shortest pathes and appropriate strains for them."""

    #copy model for less interference
    cmodel = cp.deepcopy(rs_model.cmodel)

    #create and solve shortest path model
    sp_amodel = mg.RsModelGenerator(mg.RouteConstraintsGenerator()).CreateAbstractModel()
    def ObjectiveShortestPathRule(model):
        return sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
    sp_amodel.RouteObj = pyo.Objective(rule = ObjectiveShortestPathRule, sense = pyo.minimize)
    sp_cmodel = sp_amodel.create_instance(data = rs_model.init_data)
    sp_solver = sm.GlpkSolver()
    sp_solution = sp_solver.Solve(sp_cmodel, False)    
    if sp_solution == False:
        return None
    sp_route = [ { arc : pyo.value(sp_cmodel.FlowRoute[flow, arc]) for arc in sp_cmodel.Arcs } 
                    for flow in sp_cmodel.Flows ]

    #find feasible flow strains
    if preserve_feasibility:
        feasible_solution = RecoverFeasibleStrain(rs_model, sp_route, sm.IpoptSolver())
        feasible_solution['Time'] += sp_solver.time
        return feasible_solution

    #find maximum flow
    minimum_capacity = defaultdict(lambda: float('inf'))
    for flow, node_s, node_d in cmodel.FlowRoute:
        route_val = sp_route[flow][(node_s, node_d)]
        cmodel.FlowRoute[(flow, node_s, node_d)].fix(route_val)
        if route_val >= (1 - 1e-3):
            minimum_capacity[flow] = min( [minimum_capacity[flow], cmodel.Capacity[node_s, node_d]] )
    for flow in cmodel.FlowStrain:
        cmodel.FlowStrain[flow].fix(minimum_capacity[flow])
    obj_val, strain_val, route_val = sm.isolver.ISolver.ExtractSolution(cmodel)
    max_solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': sp_solver.time}
    return max_solution