import SolverManager as sm
from .coord import Coordinator
from .gradstep import IStepRule
from .stopcrit import StopCriterion
import pyomo.environ as pyo

class CoordinatorCuttingPlane(Coordinator):
    def __init__(self, lm_min = -1, lm_max = 1):
        super().__init__()
        self.coordination_model = None
        self.coordination_components = []
        self.coordination_components_names = []
        self.obj_val_rec = []
        self.grad_rec = []
        self.lm_rec = []
        self.lm_min = lm_min
        self.lm_max = lm_max

    def InitMasterModel(self, cmodel):
        #create coordination master problem
        self.coordination_model = pyo.ConcreteModel()
        LM_MIN_VALUE, LM_MAX_VALUE = (self.lm_min, self.lm_max)
        bounds_map = { '<=': (0, LM_MAX_VALUE), '==': (LM_MIN_VALUE, LM_MAX_VALUE) }
        for names in self.relaxation_names:
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_constraint_sign = cmodel.Suffix[relaxed_constraint]['CompareName']
            relaxed_set = getattr(cmodel, names[1])
            lm_variables_len = len(relaxed_set.initialize)
            
            lm_range_name = names[2] + 'Range'
            lm_range = pyo.RangeSet(lm_variables_len)
            setattr(self.coordination_model, lm_range_name, lm_range)
            
            lm_name = names[2] + 'Var'
            lm_var_init_val = None
            if relaxed_constraint_sign == '<=':
                lm_var_init_val = self.lm_init_val_ineq
            if relaxed_constraint_sign == '==':
                lm_var_init_val = self.lm_init_val_eq
            lm = pyo.Var(lm_range, bounds = bounds_map[relaxed_constraint_sign], initialize = lm_var_init_val)
            setattr(self.coordination_model, lm_name, lm)

            self.coordination_components.append( {'Range': lm_range, 'Variable': lm} )
            self.coordination_components_names.append( {'Range': lm_range_name, 'Variable': lm_name} )
            self.coordination_components

        self.coordination_model.Z = pyo.Var()
        def CoordObjRule(model):
            return model.Z
        self.coordination_model.Obj = pyo.Objective(rule = CoordObjRule, sense = pyo.maximize)
        self.coordination_model.Cuts = pyo.ConstraintList()

    def UpgradeModel(self, amodels, relaxed_constraints_names):
        super().UpgradeModel(amodels, relaxed_constraints_names)
        self.InitMasterModel(amodels[0])

    def InitCoordination(self, cmodel):
        super().InitCoordination(cmodel)        
        self.lagr_mult_stop = []
        for names in self.relaxation_names:
            for _ in range(len(getattr(cmodel, names[1]))):
                sc = StopCriterion(1e-3, 2, 2, lambda slope, slope_req: slope >= slope_req or slope <= -slope_req)
                self.lagr_mult_stop.append(sc)

    def UpdateIterationData(self, cmodel):
        super().UpdateIterationData(cmodel)
        self.obj_val_rec.append(self.obj_val)
        self.grad_rec.append(self.gradient)
        self.lm_rec.append( [lm_value for lm_value, sign in self.lm] )

        #add cuts to the dual function linear approximation
        global_indx = 0
        expr = self.coordination_model.Z - self.obj_val_rec[self.n_iter - 1]
        for components in self.coordination_components:
            for var_indx in components['Variable']:
                expr = expr - ( ( components['Variable'][var_indx] - self.lm_rec[self.n_iter - 1][global_indx] ) * self.grad_rec[self.n_iter - 1][global_indx] )                
                global_indx += 1
        self.coordination_model.Cuts.add( expr <= 0 )

    def UpdateMultipliers(self, cmodel, master_solver):
        result = master_solver.Solve(self.coordination_model, False)
        if result is False:
            return False
        lm_updated = []
        for components in self.coordination_components:
            for var_indx in components['Variable']:
                lm_val_new = components['Variable'][var_indx].value
                lm_updated.append(lm_val_new)
        indx_total = 0
        for names in self.relaxation_names:
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                lm_updated_value = lm_updated[indx_total]
                if lm_updated_value is not None:
                    LagrangianMultipliers[indx] = lm_updated[indx_total]
                self.lagr_mult_stop[indx_total].PutValue(pyo.value(LagrangianMultipliers[indx]))
                indx_total += 1
                
        return master_solver.time

    def CheckExit(self):
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        var_stop = sum([int(sc.CheckStop()) for sc in self.var_stop_crit]) == len(self.var_stop_crit)
        if lm_stop or var_stop:
            return True

class CoordinatorCuttingPlaneProximal(CoordinatorCuttingPlane):    
    def __init__(self, lm_min = -1, lm_max = 1):
        super().__init__(lm_min, lm_max)
        self.coord_solver_failed = False
        self.dual_aprox_rec = [None]
        self.diff_real = None
        self.diff_pred = None

    def InitMasterModel(self, cmodel):
        super().InitMasterModel(cmodel)        

        for indx, names in enumerate(self.relaxation_names):
            lm_param_name = names[2] + 'Param'
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_constraint_sign = cmodel.Suffix[relaxed_constraint]['CompareName']
            lm_param_init_value = self.lm_init_val_eq if relaxed_constraint_sign == '==' else self.lm_init_val_ineq
            lm_param = pyo.Param(self.coordination_components[indx]['Range'],  mutable = True, initialize = lm_param_init_value)
            setattr(self.coordination_model, lm_param_name, lm_param)
            self.coordination_components[indx].update( {'Parameter': lm_param} )
            self.coordination_components_names[indx].update( {'Parameter': lm_param_name} )        
        obj_rule_original = self.coordination_model.Obj.rule
        def CreateObjProxRule(obj_rule_original):
            def ObjectiveDualRule(model):            
                ret_val = obj_rule_original(model)
                for names in self.coordination_components_names:
                    lm_range = getattr(model, names['Range'])
                    lm_par = getattr(model, names['Parameter'])
                    lm_var = getattr(model, names['Variable'])
                    for indx in lm_range:
                        ret_val -= 0.5 * ( lm_var[indx] - lm_par[indx] ) ** 2
                return ret_val
            return ObjectiveDualRule
        self.coordination_model.Obj.deactivate()        
        self.coordination_model.ObjProx = pyo.Objective(rule = CreateObjProxRule(obj_rule_original), sense = pyo.maximize)
        self.coordination_model.ObjProx.activate()

        self.coordination_model.dual = pyo.Suffix(direction=pyo.Suffix.IMPORT)
    
    def UpdateMultipliers(self, cmodel, master_solver):
        
        #calculate diff
        real_step = True
        m = 0.3
        if self.n_iter >= 2:
            self.diff_real = self.obj_val_rec[self.n_iter - 1] - self.obj_val_rec[self.n_iter - 2]
            self.diff_pred = self.dual_aprox_rec[self.n_iter - 1] - self.obj_val_rec[self.n_iter - 2]
            if self.diff_real <= m * self.diff_pred:
                real_step = False

        #updated lagrange multipliers proximity
        if real_step == False:
            for components in self.coordination_components:
                for indx in components['Range']:
                    components['Parameter'][indx] = components['Variable'][indx].value

        #check solver failure
        result = master_solver.Solve(self.coordination_model, False)
        if result is False:
            self.coord_solver_failed = True
            return False
        
        #update approx values
        dual_aprx = pyo.value(self.coordination_model.Obj)
        self.dual_aprox_rec.append(dual_aprx)
        
        #remove cut if dual is equal zero                
        for indx in self.coordination_model.Cuts:
            if self.coordination_model.Cuts[indx].active:
                if (abs(self.coordination_model.dual[self.coordination_model.Cuts[indx]]) <= 1e-3 and
                        len(self.coordination_model.Cuts) > 1):
                    self.coordination_model.Cuts[indx].deactivate()

        #update local problems lagrange multipliers
        lm_updated = []
        for components in self.coordination_components:
            for var_indx in components['Variable']:
                lm_val_new = components['Variable'][var_indx].value
                lm_updated.append(lm_val_new)
        indx_total = 0
        for names in self.relaxation_names:
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                lm_updated_value = lm_updated[indx_total]
                if lm_updated_value is not None:
                    LagrangianMultipliers[indx] = lm_updated[indx_total]
                self.lagr_mult_stop[indx_total].PutValue(pyo.value(LagrangianMultipliers[indx]))
                indx_total += 1
                
        return master_solver.time

    def CheckExit(self):
        if self.coord_solver_failed == True:
            return True
        else:
            return super().CheckExit()