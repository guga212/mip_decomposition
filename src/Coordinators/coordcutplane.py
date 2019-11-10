from .coord import Coordinator
from .gradstep import IStepRule
from .stopcrit import StopCriterion
import pyomo.environ as pyo

class CoordinatorCuttingPlane(Coordinator):
    def __init__(self):
        super().__init__()
        self.coordination_model = None
        self.coordination_model_instance = None
        self.coordination_components = []
        self.obj_val_rec = []
        self.grad_rec = []
        self.lm_rec = []

    def InitMasterModel(self, cmodel):
        #create coordination master problem
        self.coordination_model = pyo.ConcreteModel()
        LM_MIN_VALUE, LM_MAX_VALUE = (-2, 2)
        bounds_map = { '<=': (0, LM_MAX_VALUE), '==': (LM_MIN_VALUE, LM_MAX_VALUE) }
        coordination_names = []
        for names in self.relaxation_names:
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_constraint_sign = cmodel.Suffix[relaxed_constraint]['CompareName']
            relaxed_set = getattr(cmodel, names[1])
            lm_variables_len = len(relaxed_set.initialize)
            lm_range_name = names[2] + 'Range'
            lm_range = pyo.RangeSet(lm_variables_len)
            setattr(self.coordination_model, lm_range_name, lm_range)
            lm_name = names[2] + 'Var'
            lm = pyo.Var(lm_range, bounds = bounds_map[relaxed_constraint_sign])
            setattr(self.coordination_model, lm_name, lm)
            coordination_names.append( {'Range': lm_range_name, 'Variable': lm_name} )
        self.coordination_model.Z = pyo.Var()
        def CoordObjRule(model):
            return model.Z
        self.coordination_model.Obj = pyo.Objective(rule = CoordObjRule, sense = pyo.maximize)
        self.coordination_model.Cuts = pyo.ConstraintList()        
        #create model instance
        self.coordination_model_instance = self.coordination_model.create_instance()
        for names in coordination_names:
            lm = getattr(self.coordination_model_instance, names['Variable'])
            lm_range = getattr(self.coordination_model_instance, names['Range'])
            self.coordination_components.append( {'Range': lm_range, 'Variable': lm} )

    def UpgradeModel(self, amodels, relaxed_constraints_names):
        super().UpgradeModel(amodels, relaxed_constraints_names)
        self.InitMasterModel(amodels[0])

    def UpdateIterationData(self, cmodel):
        super().UpdateIterationData(cmodel)
        self.obj_val_rec.append(self.obj_val)
        self.grad_rec.append(self.gradient)
        self.lm_rec.append( [lm_value for lm_value, sign in self.lm] )

        global_indx = 0
        expr = self.coordination_model_instance.Z - self.obj_val_rec[self.n_iter - 1]
        for components in self.coordination_components:
            for var_indx in components['Variable']:
                expr = expr - ( ( components['Variable'][var_indx] - self.lm_rec[self.n_iter - 1][global_indx] ) * self.grad_rec[self.n_iter - 1][global_indx] )                
                global_indx += 1
        self.coordination_model_instance.Cuts.add( expr <= 0 )

    def UpdateMultipliers(self, cmodel, master_solver):
        result = master_solver.Solve(self.coordination_model_instance, False)
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

    def CheckExit(self):
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        if lm_stop:
            return True

    def Coordinate(self, cmodel, master_solver):
        if self.coordinating == False:
            self.InitCoordination(cmodel)
        self.UpdateIterationData(cmodel)
        self.UpdateMultipliers(cmodel, master_solver)
        if self.CheckExit() == True:
            return True
        return False

