import pyomo.environ as pyo
import copy as cp

class CoordinatorGradient:
    def __init__(self, amodel_local, relaxed_constraints_names):
        
        self.relaxation_names = []
        self.best_solution = None
        self.obj_prev = float('-inf')

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

        obj_val = pyo.value(cmodel.ObjDual)

        if(self.best_solution is None or self.best_solution[0] < obj_val):
            LagrangianMultipliersBestValues = {}
            for names in self.relaxation_names:
                lm_value = {}
                relaxed_set = getattr(cmodel, names[1])
                LagrangianMultipliers = getattr(cmodel, names[2])
                for indx in relaxed_set:
                    lm_value[indx] = pyo.value(LagrangianMultipliers[indx])
                LagrangianMultipliersBestValues[names[0]] = lm_value
            self.best_solution = (obj_val, LagrangianMultipliersBestValues)

        alpha = 0.001
        for names in self.relaxation_names:
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_set = getattr(cmodel, names[1])
            relaxed_constraint_expr = cmodel.Suffix[relaxed_constraint][0]
            relaxed_constraint_sign = cmodel.Suffix[relaxed_constraint][1]
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                gradient = pyo.value(relaxed_constraint_expr(cmodel, *indx))
                lm_new = pyo.value(LagrangianMultipliers[indx]) + alpha * gradient
                if relaxed_constraint_sign == '<=':
                    lm_new = max(0, lm_new)
                LagrangianMultipliers[indx] += lm_new

        epsilon = 0.001
        if( (obj_val - self.obj_prev) <= epsilon):
            return True
        self.obj_prev = obj_val

        return False

    def ResetIteration(self):
        self.best_solution = None
        self.obj_prev = float('inf')