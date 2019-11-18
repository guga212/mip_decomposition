from .heuristic import IHeuristicSolver, NodeWeight
from .isolver import ISolver
import pyomo.environ as pyo
import heapq
import copy
import time

class DHeuristicSolver(IHeuristicSolver):

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

    def Solve(self, cmodel):

        cmodel_inst = copy.deepcopy(cmodel)

        weights = self.CalculateWeights(cmodel_inst)

        self.path_flow = {}

        #algorithm runing time
        self.total_time = 0
        
        for flow in cmodel_inst.Flows:

            src = cmodel_inst.Src[flow]
            dst = cmodel_inst.Dst[flow]

            #the biggest weight for the unvisisted nodes
            start_distance_max = NodeWeight()
            start_distance_max.value = float('inf')
            distances = {node: start_distance_max for node in cmodel_inst.Nodes}

            #the smallest weight for the start node
            start_distance_min = NodeWeight()
            start_distance_min.value = float('-inf')
            start_distance_min.components = (max(cmodel_inst.Capacity.sparse_values()), [])

            #priority queue
            self.data_keeper = [(start_distance_min, src)]

            while len(self.data_keeper) > 0:

                #pop from the priority queue
                (current_distance, current_node), pop_time = self.PopValue()
                self.total_time += pop_time

                #check if the smallest value in queue is destination
                if current_node == dst:
                    break

                #process only once
                if current_distance.value > distances[current_node].value:
                    continue

                #update distances for the poped node's nieghbors
                for neighbor in cmodel_inst.NodesOut[current_node]:
                    current_weight = weights[(flow, current_node, neighbor)]
                    distance, sum_time = self.SumWeights(current_distance, current_weight)
                    self.total_time += sum_time
                    value_time = self.SetWeightValue(distance, cmodel_inst, flow)
                    self.total_time += value_time

                    #put newly calculated distance to the priority queue
                    if distance.value < distances[neighbor].value:
                        distances[neighbor] = distance
                        push_time = self.PushValue((distance, neighbor))
                        self.total_time += push_time
            #save path to the destination
            self.path_flow[flow] = distances[dst]

        #insert solution into the model
        return self.ExtractSolution(cmodel)