import ModelGenerator as mg
import ModelProcessors as mp
import SolverManager as sm
import pyomo.environ as pyo
import collections as col
import random as rnd
import copy as cp
import math as m

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

    def CreateBranchingModel(self):
        return cp.deepcopy(self.rs_model)

    def EstimateBounds(self, rs_model):
        return (None, None)

    def BranchModel(self, rs_model): 
        return []

    def Solve(self, log = True):

        #init options
        self.log = log

        #modify model
        rs_model_branching = self.CreateBranchingModel()

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

        self.LogStataus(f'#######!!!!!!!$BnB$#######!!!!!!!')
        
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
                branching_models =  self.BranchModel(bd.rs_model)
                for bm in branching_models:
                    bd = self.BranchData(bm)
                    bd.parent_indx = bd_indx
                    branching_data[branch_level+1].append(bd)
            
            #update global relaxed bounds
            if branch_level in branching_level_lb:
                prim_lb = max(prim_lb, branching_level_lb[branch_level])

            #increase explored branch level
            branch_level += 1

            #check branching stop
            if abs(prim_ub - prim_lb) <= self.required_precision:
                break
            if len(branching_data[branch_level]) == 0:
                break

        #exit algorithm
        self.LogStataus(f'###Finished[LB* = {prim_lb}, UB* = {prim_ub}, Time = {self.total_time}')
        return {'LB': prim_lb, 'UB': prim_ub, 'Time': self.total_time }

