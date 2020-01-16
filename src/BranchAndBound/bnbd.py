from .bnb import BranchAndBoundSolver
import ModelProcessors as mp
import SolverManager as sm
import pyomo.environ as pyo
import collections as col
import random as rnd
import copy as cp
import math as m

class BranchAndBoundSolverD(BranchAndBoundSolver):
    
    def __init__(self, network, flow_bounds):
        super().__init__(network, flow_bounds)
        self.dsolver = sm.DHeuristicSolver('Heap')
        self.rsolver = sm.IpoptSolver()

    def CreateBranchingModel(self):
        branching_model = cp.deepcopy(self.rs_model)
        branching_model.excluded_arcs = []
        return branching_model

    def EstimateBounds(self, rs_model):
        #solve model with dijkstra algorithm lb
        dresult = self.dsolver.Solve(rs_model.cmodel, rs_model.excluded_arcs)
        #if dijkstra solver failed
        if dresult is None:
            return (None, None)
        self.total_time += dresult['Time']
        #allocate feasible flow rate ub
        rresult = mp.RecoverFeasibleStrain(rs_model, dresult['Route'], self.rsolver)
        #if recovery failed
        if dresult is None:
            return (None, None)
        self.total_time += rresult['Time']
        return ( dresult['Objective'], rresult['Objective'] )

    def BranchModel(self, rs_model):
        branching_rs_models = []
        nonzero_accuracy = 0.01
        branch_route_len = 1
        for flow in rs_model.cmodel.Flows:
            nonzero_routes = [fr for fr_indx, fr in rs_model.cmodel.FlowRoute.items() 
                                if fr.value > nonzero_accuracy and fr_indx[0] == flow ]            
            branch_route_len = min( len(nonzero_routes), branch_route_len )
            rnd.shuffle(nonzero_routes)
            splited_routes = [ nonzero_routes[i * branch_route_len:(i + 1) * branch_route_len] 
                                for i in range((len(nonzero_routes) + branch_route_len - 1) // branch_route_len ) ]
            for sr in splited_routes:
                brs_model = cp.deepcopy(rs_model)
                for r in sr:
                    brs_model.excluded_arcs.append(r.index())
                branching_rs_models.append(brs_model)
        return branching_rs_models

     