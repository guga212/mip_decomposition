import ModelGenerator as mg
import ModelProcessors as mp
import SolverManager as sm
import pyomo.environ as pyo
import collections as col
import random as rnd
import math as m
import copy as cp

class BranchAndBoundSolver:

    class BranchData:
        def __init__(self, rs_model):
            self.rs_model = rs_model
            self.parent_indx = None
            self.lb = float('-inf')
            self.ub = float('inf')

    def __init__(self, network, flow_bounds):
        self.log = True
        self.BranchingCutsName = 'BranchingCuts'
        self.required_precision = 1e-4
        self.RouteSolver = sm.GlpkSolver()
        self.FlowRateAllocteSolver = sm.IpoptSolver()
        #get network params
        n_list = network.GetNodeList()
        f_list = network.GetFlowList()
        sd_dict = network.GetSrcDstDict()
        a_list = network.GetArcList()
        c_dict = network.GetCapacityParam()
        #create model
        nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.RouteConstraintsGenerator(), 
                                    mg.NonlinearCapacityConstraintsGenerator())
        self.rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, flow_bounds)

    def LogStataus(self, status):
        if self.log:
            print(status)
    
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
            minimum_capacity = col.defaultdict(lambda: float('inf'))
            for flow, node_s, node_d in cmodel.FlowRoute:
                route_val = routes[flow][(node_s, node_d)]
                cmodel.FlowRoute[(flow, node_s, node_d)].fix(route_val)
                if route_val >= (1 - 1e-3):
                    minimum_capacity[flow] = min( [minimum_capacity[flow], cmodel.Capacity[node_s, node_d]] )
            for flow in cmodel.FlowStrain:
                cmodel.FlowStrain[flow].fix(minimum_capacity[flow])
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

    def Solve(self, log = True):

        #init options
        self.log = log

        #modify model
        rs_model_branching = cp.deepcopy(self.rs_model)
        setattr(rs_model_branching.amodel, self.BranchingCutsName, pyo.ConstraintList())
        def ObjectiveShortestPathRule(model):
            return sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        rs_model_branching.amodel.RouteObj = pyo.Objective(rule = ObjectiveShortestPathRule, sense = pyo.minimize)
        rs_model_branching.amodel.RouteObj.deactivate()
        rs_model_branching.cmodel = rs_model_branching.amodel.create_instance(data = rs_model_branching.init_data)

        #original problems date
        prim_ub = float('inf')
        prim_lb = float('-inf')
        self.total_time = 0

        #branching proccess data keeper
        branching_data = col.defaultdict( lambda: [] )
        branching_level_lb = col.defaultdict( lambda: float('inf') )
        

        #zero level of branching
        branch_level = 0
        branching_data[branch_level] = [self.BranchData(rs_model_branching)]

        self.LogStataus(f'#######!!!!!!!BnB$#######!!!!!!!')
        
        while True:
                
            self.LogStataus(f'_____________________________:')
            self.LogStataus(f'Level: {branch_level}. Global LB: {prim_lb}. Global UB: {prim_ub}')

            #estimate bounds for current level branches
            branching_data_current = branching_data[branch_level]

            #iterate over every branch
            for bd_indx, bd in enumerate(branching_data_current):
                
                #estimate bounds for current branch
                bd.lb, bd.ub = self.EstimateBounds(bd.rs_model)
                
                self.LogStataus(f'###Branch[{bd_indx}]<-[{bd.parent_indx}]: lb = {bd.lb}, ub = {bd.ub}')

                #check exit feasibility
                if bd.lb is None or bd.ub is None:
                    continue
                
                #update global feasible bounds
                prim_ub = min(prim_ub, bd.ub)

                #update branch level relaxation bounds
                branching_level_lb[branch_level] = min(branching_level_lb[branch_level], bd.lb)

                #check exit global feasible smaller then branch relaxed
                if bd.lb >= prim_ub:
                    continue

                #check exit optimality
                if bd.lb == bd.ub:
                    continue

                #add branching
                left_branch_rs_model, right_branch_rs_model = self.BranchModel(bd.rs_model)
                bd_left = self.BranchData(left_branch_rs_model)
                bd_left.parent_indx = bd_indx
                branching_data[branch_level+1].append(bd_left)
                bd_right = self.BranchData(right_branch_rs_model)
                bd_right.parent_indx = bd_indx
                branching_data[branch_level+1].append(bd_right)
            
            #update global relaxed bounds
            prim_lb = max(prim_lb, branching_level_lb[branch_level])

            #increase explored branch level
            branch_level += 1

            #check branching stop
            if abs(prim_ub - prim_lb) <= self.required_precision:
                break
        debug_stop = 1

