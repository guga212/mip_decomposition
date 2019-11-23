import ModelGenerator as mg
from ModelGenerator.objmk import LinearObjectiveGenerator, QuadObjectiveGenerator
from ModelGenerator.constrmk import RouteConstraintsGenerator, NonlinearCapacityConstraintsGenerator
import pyomo.environ as pyo

def FindConstraintsViolation(cmodel, strain_value, route_value):
    
    flows = cmodel.Flows.value_list
    nodes = cmodel.Nodes.value_list
    nodes_in = [cmodel.NodesIn[node].value_list for node in cmodel.Nodes]
    nodes_out = [cmodel.NodesOut[node].value_list for node in cmodel.Nodes]
    arcs = cmodel.Arcs.value_list
    capacity = { arc : cmodel.Capacity[arc] for arc in arcs }
    src = [pyo.value(cmodel.Src[flow]) for flow in cmodel.Flows]
    dst = [pyo.value(cmodel.Dst[flow]) for flow in cmodel.Flows]
    
    def ConstraintRoute(flow, node):
        violations = 0
        sum_eq = -1 if node == src[flow] else 1 if node == dst[flow] else 0
        violations = + sum(route_value[flow][(i, node)] for i in nodes_in[node]) \
                     - sum(route_value[flow][(node, j)] for j in nodes_out[node]) \
                     - sum_eq
        violations = abs(violations)
        return violations
    error_boundary = 1e-6 
    route_constr_violations = [[ConstraintRoute(flow, node) for node in nodes] for flow in flows]
    route_constr_violations_nmb = sum([ sum([abs(el) >= error_boundary for el in row]) for row in route_constr_violations])

    def ConstraintCapacity(arc):
        violations = 0
        violations = + sum(strain_value[flow] * route_value[flow][arc] for flow in flows) \
                     - capacity[arc]
        return  violations
    capacity_constr_violations = { arc: ConstraintCapacity(arc) for arc in arcs }
    error_boundary = 1e-6
    capacity_constr_violations_nmb = sum( [ int(v > error_boundary) for _, v in capacity_constr_violations.items() ] )

    result = {'route_constraints' : (route_constr_violations, route_constr_violations_nmb), 
            'capacity_constraints' : (capacity_constr_violations, capacity_constr_violations_nmb),}

    return result