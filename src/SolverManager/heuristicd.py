from .isolver import ISolver
import pyomo.environ as pyo
import heapq
import copy
import time

class HeuristicSolver:

    def __init__(self, list_type):
        if list_type == 'Heap':
            self.type = 'Heap'
        elif list_type == 'List':
            self.type = 'List'
        else:
            self.type ='List'
        self.data_keeper = None
    
    def PopValue(self):
        time_elapsed = 0.0
        if self.type == 'Heap':
            start_time_pop = time.process_time()
            min_node = heapq.heappop(self.data_keeper)
            time_elapsed  = time.process_time() - start_time_pop
        if self.type == 'List':
            start_time_pop = time.process_time()
            min_node = min(self.data_keeper)
            self.data_keeper.remove(min_node)
            time_elapsed  = time.process_time() - start_time_pop
        return (min_node, time_elapsed)

    def PushValue(self, node):
        time_elapsed = 0.0
        if self.type == 'Heap':
            start_time_push = time.process_time()
            heapq.heappush(self.data_keeper, node)
            time_elapsed  = time.process_time() - start_time_push
        if self.type == 'List':
            start_time_push = time.process_time()
            self.data_keeper.append(node)
            time_elapsed  = time.process_time() - start_time_push
        return time_elapsed

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
        start_time_value = time.process_time()
        weight.value = pyo.value(cmodel.Obj)
        return time.process_time() - start_time_value

    def CalculateWeights(self, cmodel):
        weight_dict = {}
        for flow in cmodel.Flows:
            for node in cmodel.Nodes:
                for node_out in cmodel.NodesOut[node]:
                    weight_dict[(flow, node, node_out)] = self.weight()
                    weight_dict[(flow, node, node_out)].components = ( cmodel.Capacity[node, node_out], [(node, node_out)] )
        return weight_dict

    def SumWeights(self, wa, wb):
        time_elapsed = 0.0
        start_time_sum = time.process_time()
        ret_val = self.weight()
        capacity_sum = min(wa.components[0], wb.components[0])
        routes_sum = wa.components[1] + wb.components[1]
        ret_val.components = (capacity_sum, routes_sum)
        time_elapsed = time.process_time() - start_time_sum
        return (ret_val, time_elapsed)

    def Solve(self, cmodel):

        cmodel_inst = copy.deepcopy(cmodel)

        weights = self.CalculateWeights(cmodel_inst)

        path_flow = {}

        #algorithm runing time
        total_time = 0
        
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
            self.data_keeper = [(start_distance_min, src)]

            while len(self.data_keeper) > 0:
                #pop from the priority queue
                (current_distance, current_node), pop_time = self.PopValue()
                total_time += pop_time

                #check if the smallest value in queue is destination
                if current_node == dst:
                    break

                #process only once
                if current_distance.value > distances[current_node].value:
                    continue

                #update distances for the poped node's nieghbors
                for neighbor in cmodel_inst.NodesOut[current_node]:
                    weight = weights[(flow, current_node, neighbor)]
                    distance, sum_time = self.SumWeights(current_distance, weight)
                    total_time += sum_time
                    value_time = self.SetWeightValue(distance, cmodel_inst, flow)
                    total_time += value_time

                    #put newly calculated distance to the priority queue
                    if distance.value < distances[neighbor].value:
                        distances[neighbor] = distance
                        push_time = self.PushValue((distance, neighbor))
                        total_time += push_time
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