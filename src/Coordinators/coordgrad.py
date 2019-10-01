from .stopcrit import StopCriterion
import pyomo.environ as pyo
import copy as cp

class CoordinatorGradient:
    def __init__(self, amodel_local, relaxed_constraints_names):

        self.iterating = False
        self.n_iter = 0
        self.step = 0.001
        self.relaxation_names = []

        for names in relaxed_constraints_names:
            relaxed_constraint_name = names[0]
            relaxed_set_name = names[1]
            LagrangianMultipliersName = 'LagrangianMultipliers' + relaxed_constraint_name
            relaxed_set = getattr(amodel_local, relaxed_set_name)
            LagrangianMultipliers = pyo.Param(relaxed_set, mutable = True, initialize = 0)
            setattr(amodel_local, LagrangianMultipliersName, LagrangianMultipliers)
            self.relaxation_names.append((relaxed_constraint_name, relaxed_set_name, LagrangianMultipliersName))

        obj_rule = amodel_local.Obj.rule
        def ObjectiveDualRule(model):
            ret_val = obj_rule(model)
            for names in self.relaxation_names:
                relaxed_constraint = getattr(model, names[0])
                relaxed_constraint_expr = model.Suffix[relaxed_constraint][0]                
                relaxed_set = getattr(model, names[1])
                lagrange_mult = getattr(model, names[2])
                if relaxed_set.is_constructed() == False:
                    relaxed_set.construct()
                ret_val += sum( [ lagrange_mult[arg] * relaxed_constraint_expr(model, *arg) for arg in relaxed_set ] )
            return ret_val
            
        amodel_local.Obj.deactivate()        
        amodel_local.ObjDual = pyo.Objective(rule = ObjectiveDualRule, sense = pyo.minimize)
        amodel_local.ObjDual.activate()
    
    def SetBest(self, cmodel):
        if self.best_solution is None:
            return None
        for names in self.relaxation_names:
            LagrangianMultipliers = getattr(cmodel, names[2])
            relaxed_set = getattr(cmodel, names[1])
            lm_best = self.best_solution[1][names[0]]
            for indx in relaxed_set:
                LagrangianMultipliers[indx] = lm_best[indx]
        return True

    def Iterate(self, cmodel):

        #init iterations parameters
        if self.iterating == False:
            self.best_solution = ( float('-inf'), None )
            self.obj_stop_crit = StopCriterion(0.01, 4, 2, lambda slope, slope_req: slope >= slope_req)
            self.lagr_mult_stop = []
            for names in self.relaxation_names:
                for _ in range(len(getattr(cmodel, names[1]))):
                    sc = StopCriterion(0.001, 3, 1, lambda slope, slope_req: slope >= slope_req or slope <= -slope_req)
                    self.lagr_mult_stop.append(sc)
            self.iterating = True            

        #update iteration number
        self.n_iter += 1

        #get objective value
        obj_val = pyo.value(cmodel.ObjDual)
        self.obj_stop_crit.PutValue(obj_val) 

        #write down the best solution
        if(self.best_solution[0] < obj_val):
            LagrangianMultipliersBestValues = {}
            for names in self.relaxation_names:
                lm_value = {}
                relaxed_set = getattr(cmodel, names[1])
                LagrangianMultipliers = getattr(cmodel, names[2])
                for indx in relaxed_set:
                    lm_value[indx] = pyo.value(LagrangianMultipliers[indx])
                LagrangianMultipliersBestValues[names[0]] = lm_value
            self.best_solution = (obj_val, LagrangianMultipliersBestValues)

        #update multipliers
        indx_sc = 0
        for names in self.relaxation_names:
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_set = getattr(cmodel, names[1])
            relaxed_constraint_expr = cmodel.Suffix[relaxed_constraint][0]
            relaxed_constraint_sign = cmodel.Suffix[relaxed_constraint][1]
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                gradient = pyo.value(relaxed_constraint_expr(cmodel, *indx))
                lm_incr = pyo.value(LagrangianMultipliers[indx]) + self.step * gradient
                if relaxed_constraint_sign == '<=':
                    lm_incr = max(0, lm_incr)
                LagrangianMultipliers[indx] += lm_incr
                self.lagr_mult_stop[indx_sc].PutValue(pyo.value(LagrangianMultipliers[indx]))
                indx_sc += 1

        #exit condition checks
        obj_stop = self.obj_stop_crit.CheckStop()
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        if obj_stop or lm_stop:
            return True
 
        return False

    def ResetIteration(self):
        self.iterating = False
        self.n_iter = 0
        self.step = 0.001
        self.best_solution = None
        self.obj_stop_crit = None
        self.lagr_mult_stop = None
