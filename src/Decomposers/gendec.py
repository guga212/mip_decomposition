import pyomo.environ as pyo
import copy as cp
from .relax import RelaxConstraints

class GeneralDecomposer:
    
    def __init__(self, rs_model, relaxed_constraints_data, decomposed_rs_models, coordinator):
        """
        General decomposer process given model. It uses relaxed constraint data 
        to generate relaxed version of the mode. After this it decompose model in to
        the smaller models according to the decomposition group data. Coordinator 
        is used for solving decomposed models.
        """

        #abstract master model
        self.amodel_master =  cp.deepcopy(rs_model.amodel)

        #abstract local models
        self.amodels_local = [ cp.deepcopy(rsm.amodel) for rsm in decomposed_rs_models ]

        #define relaxed constraints and relax them for the master model
        relaxed_constraints_names = []
        for relaxed_constraint_data in relaxed_constraints_data:
            relaxed_constraint_name = relaxed_constraint_data[0]
            relaxed_constraint = getattr(self.amodel_master, relaxed_constraint_name)
            relaxed_constraint_set_name = relaxed_constraint_name + 'RelaxedSet'
            relaxed_constraint_range = relaxed_constraint_data[1]
            relaxed_constraint_set = pyo.Set(dimen = relaxed_constraint.dim(), initialize = relaxed_constraint_range)
            setattr(self.amodel_master, relaxed_constraint_set_name, relaxed_constraint_set)
            relaxed_constraints_names.append( (relaxed_constraint_name, relaxed_constraint_set_name) )
        RelaxConstraints(self.amodel_master, relaxed_constraints_names)

        #define relaxed constraints and relax them for the local models
        for alm in self.amodels_local:
            local_relaxed_constraints_names = []
            for relaxed_constraint_data in relaxed_constraints_data:
                relaxed_constraint_name = relaxed_constraint_data[0]
                relaxed_constraint = getattr(alm, relaxed_constraint_name)
                relaxed_constraint_set_name = relaxed_constraint_name + 'RelaxedSet'
                relaxed_constraint_range = relaxed_constraint_data[1]
                relaxed_constraint_set = pyo.Set(dimen = relaxed_constraint.dim(), initialize = relaxed_constraint_range)
                setattr(alm, relaxed_constraint_set_name, relaxed_constraint_set)
                local_relaxed_constraints_names.append( (relaxed_constraint_name, relaxed_constraint_set_name) )
            RelaxConstraints(alm, local_relaxed_constraints_names)

        #initialize coordinator
        self.coordinator = coordinator
        self.coordinator.UpgradeModel([self.amodel_master, *self.amodels_local], relaxed_constraints_names)

        #create concrete models
        self.cmodel = self.amodel_master.create_instance(data = rs_model.init_data)
        self.cmodels_local = []
        for indx, alm in enumerate(self.amodels_local):
            self.cmodels_local.append(alm.create_instance(data = decomposed_rs_models[indx].init_data))

        #get the policy for the local problem solving
        self.local_solving_manager = self.coordinator.GenerateSolvingPolicy()
        
    
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
        self.local_solvers = { cml: local_solvers[indx] for indx, cml in enumerate(self.cmodels_local) }

        def SolveLocal(cmodel_local):
            cur_solver = self.local_solvers[cmodel_local]
            solution = cur_solver.Solve(cmodel_local)
            if solution is not None:
                self.total_time += solution['Time']
            return solution

        def SolveLocalAll():
            for cml in self.cmodels_local:
                SolveLocal(cml)

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
                        if lp.is_indexed() is False:
                            lp = pyo.value(p)
                            continue
                        for index in lp:
                            lp[index] = pyo.value(p[index])

        SolveLocalAll()
        while True:
            Compose()
            coord_ret = self.coordinator.Coordinate(self.cmodel)
            if coord_ret or (self.n_iter >= self.n_iter_max):
                break
            Decompose()
            if self.local_solving_manager(SolveLocal, self.cmodels_local) == False:
                return None
            self.n_iter += 1
        self.cmodel = self.coordinator.RetrieveBest()

        ret_val = master_solver.ExtractSolution(self.cmodel)
        ret_val = { 'ObjectiveDual' : ret_val[0], 'Objective': pyo.value(self.cmodel.Obj),
                    'Strain': ret_val[1], 'Route': ret_val[2], 'Time': self.total_time }
        return ret_val