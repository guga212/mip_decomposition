from .isolver import ISolver
import pyomo.environ as pyo
import heapq
import copy
import time

class HeuristicSolver:

    class weight:
        def __init__(self, value = None, capacity = 0):
            self.value = None
            self.components = None
        
        def __lt__(self, other):
            return self.value < other.value

    def SetWeightValue(self, weight, cmodel, flow):
        for route in cmodel.FlowRoute:
            if (route[1], route[2]) in weight.components[1] and flow == route[0]:
                cmodel.FlowRoute[route].fix(1)
            else:
                cmodel.FlowRoute[route].fix(0)
        for strain in cmodel.FlowStrain:
            if strain == flow:
                cmodel.FlowStrain[strain].fix(weight.components[0])
            else:
                cmodel.FlowStrain[strain].fix(0)
        weight.value = pyo.value(cmodel.Obj)

    def CalculateWeights(self, cmodel):
        weight_dict = {}
        for flow in cmodel.Flows:
            for node in cmodel.Nodes:
                for node_out in cmodel.NodesOut[node]:
                    weight_dict[(flow, node, node_out)] = self.weight()
                    weight_dict[(flow, node, node_out)].components = ( cmodel.Capacity[node, node_out], [(node, node_out)] )
        return weight_dict

    def SumWeights(self, wa, wb):
        ret_val = self.weight()
        capacity_sum = min(wa.components[0], wb.components[0])
        routes_sum = wa.components[1] + wb.components[1]
        ret_val.components = (capacity_sum, routes_sum)
        return ret_val

    def Solve(self, cmodel):

        cmodel_inst = copy.deepcopy(cmodel)

        weights = self.CalculateWeights(cmodel_inst)

        path_flow = {}

        #start of the algorithm
        start_time = time.process_time()

        for flow in cmodel_inst.Flows:

            src = cmodel_inst.Src[flow]
            dst = cmodel_inst.Dst[flow]

            #the biggest weight for the unvisisted nodes
            start_distance_max = self.weight()
            start_distance_max.value = float('inf')
            distances = {node: start_distance_max for node in cmodel_inst.Nodes}

            #the smallest weight for the start node
            start_distance_min = self.weight()
            start_distance_min.value = float('-inf')
            start_distance_min.components = (max(cmodel_inst.Capacity.sparse_values()), [])

            #priority queue
            pq = [(start_distance_min, src)]

            while len(pq) > 0:
                #pop from the priority queue
                current_distance, current_node = heapq.heappop(pq)

                #check if the smallest value in queue is destination
                if current_node == dst:
                    break

                #process only once
                if current_distance.value > distances[current_node].value:
                    continue

                #update distances for the poped node's nieghbors
                for neighbor in cmodel_inst.NodesOut[current_node]:
                    weight = weights[(flow, current_node, neighbor)]
                    distance = self.SumWeights(current_distance, weight)
                    self.SetWeightValue(distance, cmodel_inst, flow)

                    #put newly calculated distance to the priority queue
                    if distance.value < distances[neighbor].value:
                        distances[neighbor] = distance
                        heapq.heappush(pq, (distance, neighbor))
            #save path to the destination
            path_flow[flow] = distances[dst]

        #end of the algorithm
        end_time = time.process_time()
        elapsed_time = end_time - start_time

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
        solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': elapsed_time }

        
        return solution