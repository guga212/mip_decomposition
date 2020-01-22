from .bnb import BranchAndBoundSolver
import ModelProcessors as mp
import SolverManager as sm
import pyomo.environ as pyo
import collections as col
import random as rnd
import copy as cp
import math as m

class BranchAndBoundSolverRA(BranchAndBoundSolver):
    
    def __init__(self, network, flow_bounds):
        super().__init__(network, flow_bounds)
        self.RouteSolver = sm.GlpkSolver()
        self.FlowRateAllocteSolver = sm.IpoptSolver()

    def CreateBranchingModel(self):
        rs_model_branching = cp.deepcopy(self.rs_model)
        setattr(rs_model_branching.amodel, self.BranchingCutsName, pyo.ConstraintList())
        def ObjectiveShortestPathRule(model):
            return sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        rs_model_branching.amodel.RouteObj = pyo.Objective(rule = ObjectiveShortestPathRule, sense = pyo.minimize)
        rs_model_branching.amodel.RouteObj.deactivate()
        rs_model_branching.cmodel = rs_model_branching.amodel.create_instance(data = rs_model_branching.init_data)
        return rs_model_branching

    def BranchModel(self, rs_model): 
        #define cut point       
        nonzero_accuracy = 1e-3
        drawed_floor = rnd.randint(0, len(rs_model.cmodel.Flows) - 1)
        nonzero_routes = [fr for fr_indx, fr in rs_model.cmodel.FlowRoute.items() 
                            if fr.value > nonzero_accuracy and fr_indx[0] == drawed_floor ]
        nonzero_routes_middle = m.floor( len(nonzero_routes) / 2)
        #create left branch model
        lrs_model = cp.deepcopy(rs_model)
        lmodel = lrs_model.cmodel
        lb_expr = sum([fr for fr_indx, fr in lmodel.FlowRoute.items() 
                        if fr.value > nonzero_accuracy and fr_indx[0] == drawed_floor])
        lb_branching_cuts = getattr(lmodel, self.BranchingCutsName)
        lb_branching_cuts.add( lb_expr <= nonzero_routes_middle)
        #create right branch model
        rrs_model = cp.deepcopy(rs_model)
        rmodel = rrs_model.cmodel
        rb_expr = sum([fr for fr_indx, fr in rmodel.FlowRoute.items() 
                        if fr.value > nonzero_accuracy and fr_indx[0] == drawed_floor])
        rb_branching_cuts = getattr(rmodel, self.BranchingCutsName)
        rb_branching_cuts.add( rb_expr >= nonzero_routes_middle + 1 )
        return (lrs_model, rrs_model)
    
    def __FindShortestPath__(self, cmodel):
        sp_cmodel = cp.deepcopy(cmodel)
        obj_main = [ obj for obj in sp_cmodel.component_objects(pyo.Objective, active = True) ][0]
        obj_main.deactivate()
        sp_cmodel.CapacityConstraint.deactivate()
        sp_cmodel.RouteObj.activate()
        sp_solution = self.RouteSolver.Solve(sp_cmodel, False)
        if sp_solution == False:
            return None
        sp_route = [ { arc : pyo.value(sp_cmodel.FlowRoute[flow, arc]) for arc in sp_cmodel.Arcs } 
                        for flow in sp_cmodel.Flows ]
        return {'Route': sp_route, 'Time': self.RouteSolver.time}
    
    def __AlocateFlowRates__(self, rs_model, routes, feasible):
        if feasible:
            return mp.RecoverFeasibleStrain(rs_model, routes, 
                                                self.FlowRateAllocteSolver)
        else:
            cmodel = cp.deepcopy(rs_model.cmodel)
            for flow, node_s, node_d in cmodel.FlowRoute:
                route_val = routes[flow][(node_s, node_d)]
                cmodel.FlowRoute[(flow, node_s, node_d)].fix(route_val)
            for flow in cmodel.FlowStrain:
                cmodel.FlowStrain[flow].fix(rs_model.cmodel.FlowUb[flow].value)
            obj_val, strain_val, route_val = sm.isolver.ISolver.ExtractSolution(cmodel)
            max_solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': 0}
            return max_solution

    def EstimateBounds(self, rs_model):
            
        sp_result = self.__FindShortestPath__(rs_model.cmodel)
        if sp_result is None:
            return (None, None)

        sp_route = sp_result['Route']
        for flow, node_s, node_e in rs_model.cmodel.FlowRoute:
            rs_model.cmodel.FlowRoute[flow, node_s, node_e] = sp_route[flow][(node_s, node_e)]

        self.total_time += sp_result['Time']

        lb_result = self.__AlocateFlowRates__(rs_model, sp_route, False)
        ub_result = self.__AlocateFlowRates__(rs_model, sp_route, True)
                
        ret_lb, ret_ub = None, None
        if lb_result is not None:
            ret_lb = lb_result['Objective']
            self.total_time += lb_result['Time']
        if ub_result is not None:
            ret_ub = ub_result['Objective']
            self.total_time += ub_result['Time']        
        return (ret_lb, ret_ub)
