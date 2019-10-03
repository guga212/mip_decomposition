import pyomo.environ as pyo
import copy as cp
from .relax import RelaxConstraints

class GeneralDecomposer:
    
    def __init__(self, rs_model, relaxed_constraints_data, decomposed_group_init_data, coordinator):
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
        self.coordinator = coordinator
        self.coordinator.UpgradeModel(self.amodel_local, relaxed_constraints_names)

        #create concrete models
        self.cmodel = self.amodel_local.create_instance(data = rs_model.init_data)
        self.cmodels_local = []
        for init_data_local in decomposed_group_init_data:
            self.cmodels_local.append(self.amodel_local.create_instance(data = init_data_local))

        #get the policy for the local problem solving
        self.local_solving_manager = self.coordinator.GenerateSolvingPolciy(len(self.cmodels_local))
        
    
    def Solve(self, master_solver, local_solvers):
        """
        Finds solution of the original model via 
        solving relaxed and decomposed models.
        Uses given solvers for solving decomposed 
        models and uses master solver for solving 
        coordination problem.
        """

        self.total_time = 0
        self.n_iter = 0
        self.n_iter_max = 100

        def SolveLocal(indx):
            cur_solver = local_solvers[indx]
            solution = cur_solver.Solve(self.cmodels_local[indx])
            if solution is None:
                return False
            self.total_time += solution['Time']
            return True

        def SolveLocalAll():
            for indx in range(len(self.cmodels_local)):
                SolveLocal(indx)

        def Compose():
            for cm_loc in self.cmodels_local:
                for lv in cm_loc.component_objects(pyo.Var, active = True):
                    v = getattr(self.cmodel, lv.name)
                    for index in lv:
                        v[index].fix(pyo.value(lv[index]))

        def Decompose():
            for cm_loc in self.cmodels_local:
                for lp in cm_loc.component_objects(pyo.Param, active = True):
                    if lp._mutable:
                        p = getattr(self.cmodel, lp.name)
                        for index in lp:
                            lp[index] = pyo.value(p[index])

        SolveLocalAll()
        while True:
            if self.local_solving_manager(SolveLocal) == False:
                return None
            Compose()
            coord_ret = self.coordinator.Coordinate(self.cmodel)
            Decompose()
            if coord_ret or (self.n_iter >= self.n_iter_max):
                break
            self.n_iter += 1
        self.cmodel = self.coordinator.RetrieveBest()

        ret_val = master_solver.ExtractSolution(self.cmodel)
        ret_val = { 'Objective dual' : ret_val[0], 'Objective': pyo.value(self.cmodel.Obj),
                    'Strain': ret_val[1], 'Route': ret_val[2], 'Time': self.total_time }
        return ret_val