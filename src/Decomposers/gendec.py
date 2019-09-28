import pyomo.environ as pyo
import copy as cp
from .relax import RelaxConstraints

class GeneralDecomposer:
    
    def __init__(self, rs_model, relaxed_constraints_data, decomposed_group_init_data, coordinator_cls):
        """
        General decomposer process given model. It uses relaxed constraint data 
        to generate relaxed version of the mode. After this it decompose model in to
        the smaller models according to the decomposition group data. Coordinator 
        is used for solving decomposed models.
        """

        #abstract local model
        self.amodel_local = cp.deepcopy(rs_model.amodel)

        #define relaxed constraints and relax them
        relaxed_constraints_names = []
        for relaxed_constraint_data in relaxed_constraints_data:
            relaxed_constraint_name = relaxed_constraint_data[0]
            relaxed_constraint = getattr(self.amodel_local, relaxed_constraint_name)
            relaxed_constraint_set_name = relaxed_constraint_name + 'RelaxedSet'
            relaxed_constraint_range = relaxed_constraint_data[1]
            relaxed_constraint_set = pyo.Set(dimen = relaxed_constraint.dim(), initialize = relaxed_constraint_range)
            setattr(self.amodel_local, relaxed_constraint_set_name, relaxed_constraint_set)
            relaxed_constraints_names.append( (relaxed_constraint_name, relaxed_constraint_set_name) )
        RelaxConstraints(self.amodel_local, relaxed_constraints_names)

        #initialize coordinator
        self.coordinator = coordinator_cls(self.amodel_local, relaxed_constraints_names)

        #create concrete models
        self.cmodel = self.amodel_local.create_instance(data = rs_model.init_data)
        self.cmodels_local = []
        for init_data_local in decomposed_group_init_data:
            self.cmodels_local.append(self.amodel_local.create_instance(data = init_data_local))        
    
    def Solve(self, master_solver, local_solvers):
        """
        Finds solution of the original model via 
        solving relaxed and decomposed models.
        Uses given solvers for solving decomposed 
        models and uses master solver for solving 
        coordination problem.
        """

        total_time = 0
        n_iter_max = 10
        n_iter = 0

        def SolveLocal():
            cur_solver_indx = 0
            for cm_loc in self.cmodels_local:
                if cur_solver_indx > len(local_solvers):
                    cur_solver = local_solvers[-1]
                else:
                    cur_solver = local_solvers[cur_solver_indx]
                solution = cur_solver.Solve(cm_loc)
                nonlocal total_time
                total_time += solution['Time']
                if solution is None:
                    return False
            return True

        def Compose():
            for cm_loc in self.cmodels_local:
                for lv in cm_loc.component_objects(pyo.Var, active = True):
                    v = getattr(self.cmodel, lv.name)
                    for index in lv:
                        v[index].fix(pyo.value(lv[index]))

        def Decompose():
            for cm_loc in self.cmodels_local:
                for lp in cm_loc.component_objects(pyo.Param, active=True):
                    if lp._mutable:
                        p = getattr(self.cmodel, lp.name)
                        for index in lp:
                            lp[index] = p[index]

        while True:
            if SolveLocal() is False:
                return None
            Compose()
            coord_ret = self.coordinator.Iterate(self.cmodel)
            Decompose()
            if coord_ret or (n_iter >= n_iter_max):
                break
            n_iter += 1
        self.coordinator.SetBest(self.cmodel)
        Decompose()
        SolveLocal()
        Compose()

        ret_val = master_solver.ExtractSolution(self.cmodel)
        ret_val = { 'Objective dual' : ret_val[0], 'Objective': pyo.value(self.cmodel.Obj),
                    'Strain': ret_val[1], 'Route': ret_val[2], 'Time': total_time }
        return ret_val