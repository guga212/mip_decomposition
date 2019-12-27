import pyomo.environ as pyo

class ISolver:
    @staticmethod
    def ExtractSolution(cmodel):

        if cmodel.name == 'RoutingStrainAbstractModel':
            for objective in cmodel.component_objects(pyo.Objective, active = True):
                obj_val = pyo.value(objective)
                break
            strain_val = [pyo.value(cmodel.FlowStrain[flow]) for flow in cmodel.Flows]
            route_val = [ { arc : pyo.value(cmodel.FlowRoute[flow, arc]) for arc in cmodel.Arcs } 
                            for flow in cmodel.Flows ]    
            return (obj_val, strain_val, route_val)

        if cmodel.name == 'SubnetRoutingStrainAbstractModel':
            for objective in cmodel.component_objects(pyo.Objective, active = True):
                obj_val = pyo.value(objective)
                break
            strain_val = [ max([pyo.value(cmodel.FlowStrain[flow, subnet]) for subnet in cmodel.Subnets]) for flow in cmodel.Flows ]
            route_val = []
            for flow in cmodel.Flows:
                route_val_local = {}
                for arc in cmodel.Arcs:
                    current_indx = (arc[1], arc[2])
                    if current_indx in route_val_local:
                        route_val_local[current_indx] += pyo.value(cmodel.FlowRoute[flow, arc])
                        route_val_local[current_indx] = max(min(route_val_local[current_indx], 1), 0)
                    else:
                        route_val_local[current_indx] = pyo.value(cmodel.FlowRoute[flow, arc])
                route_val.append(route_val_local)
            return (obj_val, strain_val, route_val)

        if cmodel.name == 'SeparateVariableContiniousModel':
            for objective in cmodel.component_objects(pyo.Objective, active = True):
                obj_val = pyo.value(objective)
                break
            strain_val = [pyo.value(cmodel.FlowStrain[flow]) for flow in cmodel.Flows]
            return (obj_val, strain_val, None)

        if cmodel.name == 'SeparateVariableBinaryModel':
            for objective in cmodel.component_objects(pyo.Objective, active = True):
                obj_val = pyo.value(objective)
                break
            route_val = [ { arc : pyo.value(cmodel.FlowRoute[flow, arc]) for arc in cmodel.Arcs }
                            for flow in cmodel.Flows ]
            return (obj_val, None, route_val)


        return None

    def __init__(self):
        self.solver = None
        self.solution = None
        self.solver_output = None        

    def Solve(self, cmodel, extract_solution = True):

        self.time = 0
        self.solver_output = self.solver.solve(cmodel)

        solver_ok = (self.solver_output.solver.status == pyo.SolverStatus.ok or
                        self.solver_output.solver.status == pyo.SolverStatus.warning)
        problem_solved = (self.solver_output.solver.termination_condition == pyo.TerminationCondition.optimal)
        if solver_ok and problem_solved:
            self.time = self.solver_output.Solver.Time
            if extract_solution is False:
                return True
            obj_val, strain_val, route_val = ISolver.ExtractSolution(cmodel)            
            self.solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': self.time }
        else:
            if extract_solution is False:
                return False
            self.solution = None
        return self.solution