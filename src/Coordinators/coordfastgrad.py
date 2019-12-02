from .coord import Coordinator
from .gradstep import IStepRule
from .stopcrit import StopCriterion
import SolverManager as sm
import pyomo.environ as pyo
import copy as cp

class CoordinatorFastGradient(Coordinator):
    
    def __init__(self):
        super().__init__()
        self.lm_init_val_eq = 0.0
        self.lm_init_val_ineq = 0.0
        self.upsilon = None
        self.delta = None
    
    def UpgradeModel(self, amodels, relaxed_constraints_names):
       super().UpgradeModel(amodels, relaxed_constraints_names)

       for am in amodels:
            am.Smoothness = pyo.Param(mutable = True, initialize = 0)
            dual_obj_rule = am.ObjDual.rule
            def CreateObjectiveSmoothDualRule(obj_rule_previous):
                def ObjectiveSmoothDualRule(model):
                    ret_val = obj_rule_previous(model)
                    strain_medium = (model.FlowUb - model.FlowLb) / 2
                    route_medium = 0.5

                    if hasattr(model, 'FlowStrain'):
                        for indx in model.FlowStrain.index_set():
                            ret_val += 0.5 * model.Smoothness * (model.FlowStrain[indx] - strain_medium) ** 2
                    
                    if hasattr(model, 'FlowRoute'):
                        for indx in model.FlowRoute.index_set():
                            ret_val += 0.5 * model.Smoothness * (model.FlowRoute[indx] - route_medium) ** 2

                    return ret_val
                return ObjectiveSmoothDualRule

            am.ObjDual.deactivate()        
            am.ObjSmoothDual = pyo.Objective(rule = CreateObjectiveSmoothDualRule(dual_obj_rule), sense = pyo.minimize)
            am.ObjSmoothDual.activate()
    
    def InitCoordination(self, cmodel):
        super().InitCoordination(cmodel)

        self.R2 = None
        def ProximalRule(model):
            ret_val = 0
            strain_medium = (model.FlowUb - model.FlowLb) / 2
            route_medium = 0.5
            for indx in model.FlowStrain.index_set():
                ret_val += 0.5 * (model.FlowStrain[indx] - strain_medium) ** 2
            for indx in model.FlowRoute.index_set():
                ret_val += 0.5 * (model.FlowRoute[indx] - route_medium) ** 2
            return ret_val
        prox_max_model = cp.deepcopy(cmodel)
        for v in prox_max_model.component_objects(pyo.Var, active = True):
            for indx in v:
                v[indx].unfix()
        prox_max_model.ObjSmoothDual.deactivate()
        prox_max_model.ObjProxMax = pyo.Objective(rule = ProximalRule, sense = pyo.maximize)
        prox_max_model.ObjProxMax.activate()
        solver = sm.minlpsolvers.CouenneSolver()
        result = solver.Solve(prox_max_model, False)
        if result is not None:
            self.R2 = pyo.value(prox_max_model.ObjProxMax)

        self.Lmu = 0.0
        self.dprev = 0.0
        self.B = 0
        self.M = 0
        self.lm_initial = []
        for names in self.relaxation_names:
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_set = getattr(cmodel, names[1])
            relaxed_constraint_expr_lhs = cmodel.Suffix[relaxed_constraint]['LHS']
            relaxed_constraint_expr_rhs = cmodel.Suffix[relaxed_constraint]['RHS']
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                self.B += pyo.value(relaxed_constraint_expr_lhs(cmodel, *indx)) ** 2
                self.M += pyo.value(relaxed_constraint_expr_rhs(cmodel, *indx)) ** 2
                self.lm_initial.append(pyo.value(LagrangianMultipliers[indx]))
        # self.M = 0.0
        # self.B = 1.0
    def UpdateMultipliers(self, cmodel):
        f_ref = -1.369
        epsilon_error = 0.99
        smoothness_updated = max( f_ref - self.best_solution[0], epsilon_error * abs(f_ref) )/ (2 * self.R2)
        cmodel.Smoothness = smoothness_updated

        k = self.n_iter - 1
        def ParamMaker(arg):
            if arg < 0:
                return (0, 0, 0)
            u = (arg + 1) / 2
            d = (arg + 1) * (arg + 2) / 2
            i = u / d
            return (u, d, i)
        upsilon, delta, iota = {}, {}, {}
        upsilon[k - 1], delta[k - 1], iota[k - 1] = ParamMaker(k - 1)
        upsilon[k], delta[k], iota[k] = ParamMaker(k)
        upsilon[k + 1], delta[k + 1], iota[k + 1] = ParamMaker(k + 1)

        self.Lmu = self.M + self.B/ cmodel.Smoothness.value
        lm_updated = []
        for indx, (lm_value, sign) in enumerate(self.lm):
            pi = lm_value + self.gradient[indx] / self.Lmu
            dzeta = self.lm_initial[indx] + delta[k - 1] *  self.dprev / self.Lmu
            d = iota[k] * self.gradient[indx] + (1 - iota[k]) * self.dprev
            self.dprev = d
            if sign == '<=':
                pi = max(0, pi)
                dzeta = max(0, dzeta)
            lm_updated_value = iota[k + 1] * dzeta + (1 - iota[k + 1]) * pi
            lm_updated.append(lm_updated_value)

        indx_total = 0
        for names in self.relaxation_names:
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                LagrangianMultipliers[indx] = lm_updated[indx_total]
                self.lagr_mult_stop[indx_total].PutValue(pyo.value(LagrangianMultipliers[indx]))
                indx_total += 1
        


    def CheckExit(self):
        self.obj_stop_crit.CheckStop()
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        if lm_stop:
            return True