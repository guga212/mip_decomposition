from .isolver import ISolver
import pyomo.environ as pyo
import heapq
import copy
import time

class NodeWeight:
    def __init__(self, value = None):
        self.value = None
        self.components = None
    
    def __lt__(self, other):
        return self.value < other.value

class IHeuristicSolver:

    def SetWeightValue(self, node_weight, cmodel, flow):
        for route in cmodel.FlowRoute:
            if (route[1], route[2]) in node_weight.components[1] and flow == route[0]:
                cmodel.FlowRoute[route].fix(1)
            else:
                cmodel.FlowRoute[route].fix(0)
        for strain in cmodel.FlowStrain:
            if strain == flow:
                cmodel.FlowStrain[strain].fix(node_weight.components[0])
            else:
                cmodel.FlowStrain[strain].fix(0)
        if node_weight.components[0] < cmodel.FlowLb.value or node_weight.components[0] > cmodel.FlowUb.value:
            node_weight.value = float('inf')
            return 0
        
        start_time_value = time.process_time()
        objective_function = [obj for obj in cmodel.component_objects(pyo.Objective, active = True)][0]
        node_weight.value = pyo.value(objective_function)
        return time.process_time() - start_time_value

    def CalculateWeights(self, cmodel):
        weight_dict = {}
        for flow in cmodel.Flows:
            for node in cmodel.Nodes:
                for node_out in cmodel.NodesOut[node]:
                    weight_dict[(flow, node, node_out)] = NodeWeight()
                    weight_dict[(flow, node, node_out)].components = ( cmodel.Capacity[node, node_out], [(node, node_out)] )
        return weight_dict

    def SumWeights(self, wa, wb):
        time_elapsed = 0.0
        start_time_sum = time.process_time()
        ret_val = NodeWeight()
        capacity_sum = min(wa.components[0], wb.components[0])
        routes_sum = wa.components[1] + wb.components[1]
        ret_val.components = (capacity_sum, routes_sum)
        time_elapsed = time.process_time() - start_time_sum
        return (ret_val, time_elapsed)

    def Solve(self, cmodel):
        self.path_flow = {}
        self.total_time = 0.0

    def ExtractSolution(self, cmodel):

        #no solution was found
        if len(self.path_flow) == 0:
            return None
        if any( [ pf.components is None or pf.value == float('inf') for _, pf in self.path_flow.items() ] ):
            return None

        #put found pathes into the original model
        for route in cmodel.FlowRoute:
            if (route[1], route[2]) in self.path_flow[route[0]].components[1]:
                cmodel.FlowRoute[route].fix(1)
            else:
                cmodel.FlowRoute[route].fix(0)
        for strain in cmodel.FlowStrain:
                cmodel.FlowStrain[strain].fix(self.path_flow[strain].components[0])

        obj_val, strain_val, route_val = ISolver.ExtractSolution(cmodel)

        #create standard solver output
        solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': self.total_time }
        
        return solution