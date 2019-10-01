import pyomo.environ as pyo

class ISolver:
    @staticmethod
    def ExtractSolution(cmodel):
        for objective in cmodel.component_objects(pyo.Objective, active = True):
            obj_val = pyo.value(objective)
            break
        strain_val = [pyo.value(cmodel.FlowStrain[flow]) for flow in cmodel.Flows]
        route_val = [ { arc : pyo.value(cmodel.FlowRoute[flow, arc]) for arc in cmodel.Arcs } 
                        for flow in cmodel.Flows ]    
        return (obj_val, strain_val, route_val)

    def __init__(self):
        self.solver = None
        self.solution = None
        self.solver_output = None

    def Solve(self, cmodel):
        self.solver_output = self.solver.solve(cmodel)

        solver_ok = (self.solver_output.solver.status == pyo.SolverStatus.ok or
                        self.solver_output.solver.status == pyo.SolverStatus.warning)
        problem_solved = (self.solver_output.solver.termination_condition == pyo.TerminationCondition.optimal or
                            self.solver_output.solver.termination_condition == pyo.TerminationCondition.globallyOptimal or
                            self.solver_output.solver.termination_condition == pyo.TerminationCondition.locallyOptimal or
                            self.solver_output.solver.termination_condition == pyo.TerminationCondition.feasible)

        if solver_ok and problem_solved:
            obj_val, strain_val, route_val = ISolver.ExtractSolution(cmodel)
            time = self.solver_output.Solver.Time
            self.solution = { 'Objective': obj_val, 'Strain': strain_val, 'Route': route_val, 'Time': time }
        else:
            self.solution = None
        return self.solution