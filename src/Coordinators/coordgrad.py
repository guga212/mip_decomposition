from .gradstep import IStepRule
from .stopcrit import StopCriterion
import pyomo.environ as pyo
import copy as cp

class CoordinatorGradient:

    def __init__(self, step_rule = IStepRule()):

        self.iterating = False
        self.n_iter = 0
        self.step_rule = step_rule
        self.relaxation_names = []
    
    def UpgradeModel(self, amodels, relaxed_constraints_names):

        for names in relaxed_constraints_names:
            relaxed_constraint_name = names[0]
            relaxed_set_name = names[1]
            LagrangianMultipliersName = 'LagrangianMultipliers' + relaxed_constraint_name
            self.relaxation_names.append((relaxed_constraint_name, relaxed_set_name, LagrangianMultipliersName))

        for am in amodels:
            for names in self.relaxation_names:
                relaxed_set = getattr(am, names[1])
                LagrangianMultipliers = pyo.Param(relaxed_set, mutable = True, initialize = 0)
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
                sc = StopCriterion(0.0001, 2, 2, lambda slope, slope_req: slope >= slope_req or slope <= -slope_req)
                self.var_stop_crit.append(sc)

        self.iterating = True

    def SetBestSolution(self, cmodel):
        LagrangianMultipliersBestValues = {}
        for names in self.relaxation_names:
            lm_value = {}
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                lm_value[indx] = pyo.value(LagrangianMultipliers[indx])
            LagrangianMultipliersBestValues[names[0]] = lm_value
        self.best_solution = (pyo.value(cmodel.ObjDual), LagrangianMultipliersBestValues, cp.deepcopy(cmodel))

    def UpdateIterationData(self, cmodel):
        obj_val = pyo.value(cmodel.ObjDual)

        #updated stop crit.
        self.obj_stop_crit.PutValue(obj_val) 
        index_vsc = 0
        for v in cmodel.component_objects(pyo.Var, active = True):
            for indx in v:
                self.var_stop_crit[index_vsc].PutValue(pyo.value(v[indx]))
                index_vsc += 1

        #write down the best solution
        if(self.best_solution[0] < obj_val):
            self.SetBestSolution(cmodel)

        #update iteration
        self.n_iter += 1
                
        #extract dual variables and gradient
        gradient = []
        lm = []
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
                gradient.append(grad_val)
                #dual variables
                lm_val = ( pyo.value(LagrangianMultipliers[indx]), relaxed_constraint_sign )
                lm.append(lm_val)

        #update step data
        self.step_rule.UpdateData(obj = obj_val, gradient = gradient, variables = lm)

    def UpdateMultipliers(self, cmodel):
        #get new dual variables
        lm_updated = self.step_rule.GetVariables()

        indx_total = 0
        for names in self.relaxation_names:
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                LagrangianMultipliers[indx] = lm_updated[indx_total]
                self.lagr_mult_stop[indx_total].PutValue(pyo.value(LagrangianMultipliers[indx]))
                indx_total += 1

    def CheckExit(self):
        obj_stop = self.obj_stop_crit.CheckStop()
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        if obj_stop or lm_stop:
            return True

    def Coordinate(self, cmodel):
        if self.iterating == False:
            self.InitCoordination(cmodel)
        self.UpdateIterationData(cmodel)
        self.UpdateMultipliers(cmodel)
        if self.CheckExit() == True:
            return True
        return False

    def ResetCoordination(self):
        self.iterating = False
        self.n_iter = 0
        self.step = 0.001
        self.best_solution = None
        self.obj_stop_crit = None
        self.var_stop_crit = None
        self.lagr_mult_stop = None
