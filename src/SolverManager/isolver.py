import pyomo.environ as pyo

class ISolver:
    def __init__(self):
        self.solver = None
    def Solve(self, model_instance):
        solver_output= self.solver.solve(model_instance)

        
        if (solver_output.solver.status != pyo.SolverStatus.ok) or (solver_output.solver.termination_condition ==  pyo.TerminationCondition.infeasible):
            obj_val = None            
            strain_val = None 
            route_val = None 
            solver_output = None
        else:
            obj_val = pyo.value(model_instance.Obj)
            strain_val = [pyo.value(model_instance.FlowStrain[flow]) for flow in model_instance.Flows]
            route_val =[ { arc : pyo.value(model_instance.FlowRoute[flow, arc]) for arc in model_instance.Arcs } for flow in model_instance.Flows ]
        return (obj_val, strain_val, route_val, solver_output)