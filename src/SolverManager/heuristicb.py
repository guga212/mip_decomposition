from .heuristic import IHeuristicSolver, NodeWeight
from .isolver import ISolver
import pyomo.environ as pyo
import copy
import time

class BHeuristicSolver(IHeuristicSolver):

    def Solve(self, cmodel):

        cmodel_inst = copy.deepcopy(cmodel)
        weights = self.CalculateWeights(cmodel_inst)

        #results of the algorithm
        path_flow = {}
    
        #algorithm runing time
        total_time = 0


        #solve every flow subproblem
        for flow in cmodel_inst.Flows:

            #retrieve source and destination node
            src = cmodel_inst.Src[flow]
            dst = cmodel_inst.Dst[flow]

            #the biggest weight for the unvisisted nodes
            start_distance_max = NodeWeight()
            start_distance_max.value = float('inf')            
        
            #the smallest weight for the start node
            start_distance_min = NodeWeight()
            start_distance_min.value = float('-inf')
            start_distance_min.components = (max(cmodel_inst.Capacity.sparse_values()), [])

            #initialize distances
            distances = {node: start_distance_max for node in cmodel_inst.Nodes}
            distances[src] = start_distance_min
            
            #maximal outer iteration number is equal to the nodes number
            for _ in cmodel_inst.Nodes:
                #recalculate distance for every edge
                for node_s, node_e in cmodel_inst.Arcs:
                    current_distance = distances[node_s]
                    if current_distance.value == float('inf'):
                        continue
                    current_weight = weights[(flow, node_s, node_e)]
                    new_distance, sum_time = self.SumWeights(current_distance, current_weight)
                    value_time = self.SetWeightValue(new_distance, cmodel_inst, flow)
                    total_time += sum_time + value_time
                    if new_distance.value < distances[node_e].value:                        
                        distances[node_e] = new_distance
            #save path to the destination                        
            path_flow[flow] = distances[dst]
            
        #put found pathes into the original model
        for route in cmodel.FlowRoute:
            if (route[1], route[2]) in path_flow[route[0]].components[1]:
                cmodel.FlowRoute[route].fix(1)
            else:
                cmodel.FlowRoute[route].fix(0)
        for strain in cmodel.FlowStrain:
                cmodel.FlowStrain[strain].fix(path_flow[strain].components[0])

        obj_val, strain_val, route_val = ISolver.ExtractSolution(cmodel)

        #create standard solver output
        solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': total_time }
        
        return solution