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
        self.coordinator.SetParams(init_data_master = rs_model.init_data)

        #create concrete models
        self.cmodel = self.amodel_master.create_instance(data = rs_model.init_data)
        self.cmodels_local = []
        for indx, alm in enumerate(self.amodels_local):
            self.cmodels_local.append(alm.create_instance(data = decomposed_rs_models[indx].init_data))

        #get the policy for the local problem solving
        self.local_solving_manager = self.coordinator.GenerateSolvingPolicy()

        #initialize data recorder
        class DataRecorder:
            def __init__(self):
                self.master_obj_value_list = None
                self.local_obj_value_list = None
                self.multipliers_dict = None
        self.data_recorded = DataRecorder()
        self.data_recorded.master_obj_value_list = []
        self.data_recorded.local_obj_value_list = [[] for _ in self.cmodels_local]
        self.data_recorded.multipliers_dict = {}
  
    @property
    def RecordedData(self): 
        return { 'MasterObj': self.data_recorded.master_obj_value_list,
                    'LocalObj': self.data_recorded.local_obj_value_list,
                    'Multipliers': self.data_recorded.multipliers_dict,
                }

    def Solve(self, master_solver, local_solvers, max_iteration = 100):
        """
        Finds solution of the original model via 
        solving relaxed and decomposed models.
        Uses given solvers for solving decomposed 
        models and uses master solver for solving 
        coordination problem.
        """

        self.total_time = 0
        self.coordination_time = 0
        self.local_times = {}
        self.n_iter = 0
        self.n_iter_max = max_iteration
        self.local_solvers = { cml: local_solvers[indx] for indx, cml in enumerate(self.cmodels_local) }

        def SolveLocal(cmodel_local):
            cur_solver = self.local_solvers[cmodel_local]
            solution = cur_solver.Solve(cmodel_local)
            if solution is not None:
                self.total_time += solution['Time']
                if self.n_iter not in self.local_times:
                    self.local_times[self.n_iter] = []
                self.local_times[self.n_iter].append(solution['Time'])
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
                            lp.value = pyo.value(p)
                            continue
                        for index in lp:
                            lp[index] = pyo.value(p[index])

        def CollectData(output_data_obj_master = False, output_data_obj_local = False, output_data_params = False, output_times = False):
            #loger function
            def LogCollectedData(log_str):
                prefix = f'#Iteration: {self.n_iter} #'
                print(prefix + log_str)
            #collect objectives values
            obj_master = [ obj for obj in self.cmodel.component_objects(pyo.Objective, active = True) ][0]
            obj_master_value = pyo.value(obj_master)
            self.data_recorded.master_obj_value_list.append( obj_master_value )
            if output_data_obj_master:
                LogCollectedData(f'MASTER OBJ VALUE: {obj_master_value}')
            for indx, cm_loc in enumerate(self.cmodels_local):
                obj_local = [ obj for obj in cm_loc.component_objects(pyo.Objective, active = True) ][0]
                obj_local_val = pyo.value(obj_local)
                self.data_recorded.local_obj_value_list[indx].append( obj_local_val )
                if output_data_obj_local:
                    LogCollectedData(f'LOCAL[{indx}] OBJ VALUE: {obj_local_val}')
            #collect Lagrangian multipliers
            for p in self.cmodel.component_objects(pyo.Param, active = True):
                if p._mutable and 'Lagrangian' in p.name:
                    if p.name not in self.data_recorded.multipliers_dict:
                        self.data_recorded.multipliers_dict[p.name] = []
                    pv_list = [pv.value for pv in p.values()]
                    self.data_recorded.multipliers_dict[p.name].append(pv_list)
                    if output_data_params:
                        LogCollectedData(f'PARAM <{p.name}>: {pv_list}')
            #collect spent times
            if output_times:
                sum_time = sum(self.local_times[self.n_iter]) + self.coordination_time                 
                LogCollectedData(f'SPENT TIME. COORDINATION:[{self.coordination_time}] + LOCAL:{self.local_times[self.n_iter ]} := {sum_time}')


        SolveLocalAll()
        while True:
            Compose()
            CollectData(output_data_obj_master = True, output_data_obj_local = False, 
                        output_data_params = False, output_times = True)
            coord_ret =  self.coordinator.Coordinate(self.cmodel, master_solver)
            self.coordination_time = coord_ret['Time'] if coord_ret['Time'] is not None else 0
            self.total_time += self.coordination_time
            if coord_ret['Terminate'] == True or (self.n_iter >= self.n_iter_max):
                break
            self.n_iter += 1
            Decompose()
            if self.local_solving_manager(SolveLocal, self.cmodels_local) == False:
                return None
        self.cmodel = self.coordinator.RetrieveBest()

        ret_val = master_solver.ExtractSolution(self.cmodel)
        ret_val = { 'ObjectiveDual' : pyo.value(self.cmodel.ObjDual), 'Objective': pyo.value(self.cmodel.Obj),
                    'Strain': ret_val[1], 'Route': ret_val[2], 'Time': self.total_time }
        return ret_val