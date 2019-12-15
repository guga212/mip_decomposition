from .gradstep import IStepRule
from .stopcrit import StopCriterion
import pyomo.environ as pyo
import copy as cp

class Coordinator:

    def __init__(self):
        self.coordinating = False
        self.n_iter = 0
        self.relaxation_names = []
        self.lm_init_val_eq = 0.0
        self.lm_init_val_ineq = 0.0
    
    def UpgradeModel(self, amodels, relaxed_constraints_names):

        for names in relaxed_constraints_names:
            relaxed_constraint_name = names[0]
            relaxed_set_name = names[1]
            LagrangianMultipliersName = 'LagrangianMultipliers' + relaxed_constraint_name
            self.relaxation_names.append((relaxed_constraint_name, relaxed_set_name, LagrangianMultipliersName))

        for am in amodels:
            for names in self.relaxation_names:
                relaxed_set = getattr(am, names[1])
                relaxed_constraint = getattr(am, names[0])
                relaxed_constraint_sign = am.Suffix[relaxed_constraint]['CompareName']
                if relaxed_constraint_sign == '<=':
                    LagrangianMultipliers = pyo.Param(relaxed_set, mutable = True, initialize = self.lm_init_val_ineq)
                if relaxed_constraint_sign == '==':
                    LagrangianMultipliers = pyo.Param(relaxed_set, mutable = True, initialize = self.lm_init_val_eq)
                setattr(am, names[2], LagrangianMultipliers)
            
            obj_rule = am.Obj.rule
            def CreateObjectiveDualRule(obj_rule_original):
                def ObjectiveDualRule(model):
                    ret_val = obj_rule_original(model)
                    for names in self.relaxation_names:
                        relaxed_constraint = getattr(model, names[0])
                        relaxed_constraint_expr_lhs = model.Suffix[relaxed_constraint]['LHS']
                        relaxed_constraint_expr_rhs = model.Suffix[relaxed_constraint]['RHS']
                        relaxed_set = getattr(model, names[1])
                        lagrange_mult = getattr(model, names[2])
                        if relaxed_set.is_constructed() == False:
                            relaxed_set.construct()
                        ret_val += sum( [ lagrange_mult[arg] * (relaxed_constraint_expr_lhs(model, *arg) -
                                                                    relaxed_constraint_expr_rhs(model, *arg))
                                            for arg in relaxed_set ] )
                    return ret_val
                return ObjectiveDualRule
            
            am.Obj.deactivate()        
            am.ObjDual = pyo.Objective(rule = CreateObjectiveDualRule(obj_rule), sense = pyo.minimize)
            am.ObjDual.activate()

    def SetParams(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

    def GenerateSolvingPolicy(self):
        def SolvingPolicy(solve, cmodels_local):
            for cml in cmodels_local:
                if solve(cml) is None:
                    return False
            return True
        self.solving_policy = SolvingPolicy
        return self.solving_policy

    def RetrieveBest(self):
        if self.best_solution is None:
            return None
        return self.best_solution[2]

    def InitCoordination(self, cmodel):
        self.best_solution = ( float('-inf'), None )
        self.obj_stop_crit = StopCriterion(0.001, 4, 2, lambda slope, slope_req: slope >= slope_req)
        self.lagr_mult_stop = []
        for names in self.relaxation_names:
            for _ in range(len(getattr(cmodel, names[1]))):
                sc = StopCriterion(0.0001, 2, 1, lambda slope, slope_req: slope >= slope_req or slope <= -slope_req)
                self.lagr_mult_stop.append(sc)
        self.var_stop_crit = []
        for v in cmodel.component_objects(pyo.Var, active = True):
            for _ in v:
                sc = StopCriterion(0.0001, 2, 6, lambda slope, slope_req: slope >= slope_req or slope <= -slope_req)
                self.var_stop_crit.append(sc)
        self.coordinating = True

    def SetBestSolution(self, cmodel):
        LagrangianMultipliersBestValues = {}
        for names in self.relaxation_names:
            lm_value = {}
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                lm_value[indx] = pyo.value(LagrangianMultipliers[indx])
            LagrangianMultipliersBestValues[names[0]] = lm_value
        obj_active = [ obj for obj in cmodel.component_objects(pyo.Objective, active = True) ][0]
        self.best_solution = (pyo.value(obj_active), LagrangianMultipliersBestValues, cp.deepcopy(cmodel))

    def UpdateIterationData(self, cmodel):
        #update iteration
        self.n_iter += 1

        #retrieve dual value
        obj_active = [ obj for obj in cmodel.component_objects(pyo.Objective, active = True) ][0]
        self.obj_val = pyo.value(obj_active)

        #updated stop crit.
        self.obj_stop_crit.PutValue(self.obj_val) 
        index_vsc = 0
        for v in cmodel.component_objects(pyo.Var, active = True):
            for indx in v:
                self.var_stop_crit[index_vsc].PutValue(pyo.value(v[indx]))
                index_vsc += 1

        #write down the best solution
        if(self.best_solution[0] < self.obj_val):
            self.SetBestSolution(cmodel)
                        
        #extract dual variables and gradient
        self.gradient = []
        self.lm = []
        for names in self.relaxation_names:
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_set = getattr(cmodel, names[1])
            relaxed_constraint_expr_lhs = cmodel.Suffix[relaxed_constraint]['LHS']
            relaxed_constraint_expr_rhs = cmodel.Suffix[relaxed_constraint]['RHS']
            relaxed_constraint_sign = cmodel.Suffix[relaxed_constraint]['CompareName']
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                #gradient
                grad_val = pyo.value(relaxed_constraint_expr_lhs(cmodel, *indx) - 
                                        relaxed_constraint_expr_rhs(cmodel, *indx))
                self.gradient.append(grad_val)
                #dual variables
                lm_val = ( pyo.value(LagrangianMultipliers[indx]), relaxed_constraint_sign )
                self.lm.append(lm_val)

    def UpdateMultipliers(self, cmodel):
        pass

    def CheckExit(self):
        obj_stop = self.obj_stop_crit.CheckStop()
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        var_stop = sum([int(sc.CheckStop()) for sc in self.var_stop_crit]) == len(self.var_stop_crit)
        if obj_stop or lm_stop or var_stop:
            return True

    def Coordinate(self, cmodel, master_solver):
        if self.coordinating == False:
            self.InitCoordination(cmodel)
        self.UpdateIterationData(cmodel)
        self.UpdateMultipliers(cmodel)
        if self.CheckExit() == True:
            return True
        return False
