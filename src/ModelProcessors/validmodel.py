import pyomo.environ as pyo

def FindConstraintsViolation(model):
    """ 
    Function for model solution validation.
    Returns dictionary of constraints violations
    status and violations values. Negative values 
    of the violations means slack.
    """

    flows = model.Flows.value_list
    nodes = model.Nodes.value_list
    nodes_in = [model.NodesIn[node].value_list for node in model.Nodes]
    nodes_out = [model.NodesOut[node].value_list for node in model.Nodes]
    arcs = model.Arcs.value_list
    capacity = { arc : model.Capacity[arc] for arc in arcs }
    src = [pyo.value(model.Src[flow]) for flow in model.Flows]
    dst = [pyo.value(model.Dst[flow]) for flow in model.Flows]
    strain_val = [pyo.value(model.FlowStrain[flow]) for flow in model.Flows]
    route_val =[ { arc : pyo.value(model.FlowRoute[flow, arc]) for arc in model.Arcs } for flow in model.Flows ]

    def ConstraintRoute(flow, node):
        violations = 0
        sum_eq = -1 if node == src[flow] else 1 if node == dst[flow] else 0
        violations = + sum(route_val[flow][(i, node)] for i in nodes_in[node]) \
                     - sum(route_val[flow][(node, j)] for j in nodes_out[node]) \
                     - sum_eq
        violations = abs(violations)
        return violations
    route_constr_violations = [[ConstraintRoute(flow, node) for node in nodes] for flow in flows]
    route_constr_violations_nmb = sum([ sum([el != 0 for el in row]) for row in route_constr_violations])

    def ConstraintCapacity(arc):
        violations = 0
        violations = + sum(strain_val[flow] * route_val[flow][arc] for flow in flows) \
                     - capacity[arc]
        return  violations
    capacity_constr_violations = [ConstraintCapacity(arc) for arc in arcs]
    capacity_constr_violations_nmb = sum( [ int(v > 0) for v in capacity_constr_violations ] )

    result = {'route_constraints' : (route_constr_violations, route_constr_violations_nmb), 
            'capacity_constraints' : (capacity_constr_violations, capacity_constr_violations_nmb),}

    return result

        