from .coord import Coordinator
from .gradstep import IStepRule
from .stopcrit import StopCriterion
import SolverManager as sm
import ModelProcessors as mp
import ModelGenerator as mg
import pyomo.environ as pyo
import copy as cp

class CoordinatorFastGradient(Coordinator):

    def __init__(self, optimal_obj = None):
        super().__init__()
        self.optimal_obj = optimal_obj
        self.lm_init_val_eq = 0.0
        self.lm_init_val_ineq = 0.0
        self.upsilon = None
        self.delta = None

    def UpgradeModel(self, amodels, relaxed_constraints_names):
        self.amodel_master = cp.deepcopy(amodels[0])
        self.init_data_master = None

        super().UpgradeModel(amodels, relaxed_constraints_names)

        for am in amodels:
            am.Smoothness = pyo.Param(mutable = True, initialize = 0)

            if hasattr(am, 'FlowStrain'):
                def flow_center_param_init(model, *args):
                    return model.FlowLb + (model.FlowUb - model.FlowLb) * 0.5
                flow_center_param = pyo.Param(am.FlowStrain.index_set(), mutable = True, initialize = flow_center_param_init)
                setattr(am, 'FlowStrainCenter', flow_center_param)
            if hasattr(am, 'FlowRoute'):
                route_center_param = pyo.Param(am.FlowRoute.index_set(), mutable = True, initialize = 0.5)
                setattr(am, 'FlowRouteCenter', route_center_param)
            if hasattr(am, 'FlowStrainMulRoute'):
                def artvar_center_param_init(model, *args):
                    return (model.FlowLb + (model.FlowUb - model.FlowLb) * 0.5)
                artvar_center_param = pyo.Param(am.FlowStrainMulRoute.index_set(), mutable = True, initialize = artvar_center_param_init)
                setattr(am, 'FlowStrainMulRouteCenter', artvar_center_param)


            dual_obj_rule = am.ObjDual.rule
            def CreateObjectiveSmoothDualRule(obj_rule_previous):
                def ObjectiveSmoothDualRule(model):
                    ret_val = obj_rule_previous(model)
                    
                    if hasattr(model, 'FlowStrain'):
                        for indx in model.FlowStrain.index_set():
                            ret_val += 0.5 * model.Smoothness * (model.FlowStrain[indx] - model.FlowStrainCenter[indx]) ** 2

                    if hasattr(model, 'FlowRoute'):
                        for indx in model.FlowRoute.index_set():
                            ret_val += 0.5 * model.Smoothness * (model.FlowRoute[indx] - model.FlowRouteCenter[indx]) ** 2

                    if hasattr(model, 'FlowStrainMulRoute'):
                        for indx in model.FlowStrainMulRoute.index_set():
                            ret_val += 0.5 * model.Smoothness * (model.FlowStrainMulRoute[indx] - model.FlowStrainMulRouteCenter[indx]) ** 2

                    return ret_val
                return ObjectiveSmoothDualRule

            am.ObjDual.deactivate()
            am.ObjSmoothDual = pyo.Objective(rule = CreateObjectiveSmoothDualRule(dual_obj_rule), sense = pyo.minimize)
            am.ObjSmoothDual.activate()

    def InitCoordination(self, cmodel):
        super().InitCoordination(cmodel)

        #change default Lagrange multipliers stop criterion
        self.lagr_mult_stop = []
        for names in self.relaxation_names:
            for _ in range(len(getattr(cmodel, names[1]))):
                sc = StopCriterion(1e-4, 2, 5, lambda slope, slope_req: slope >= slope_req or slope <= -slope_req)
                self.lagr_mult_stop.append(sc)

        #find maximum of the proximal operator
        def ProximalRule(model):
            ret_val = 0
            for indx in model.FlowStrain.index_set():
                ret_val += 0.5 * (model.FlowStrain[indx] - model.FlowStrainCenter[indx]) ** 2
            for indx in model.FlowRoute.index_set():
                ret_val += 0.5 * (model.FlowRoute[indx] - model.FlowRouteCenter[indx]) ** 2
            if hasattr(model, 'FlowStrainMulRoute'):
                for indx in model.FlowStrainMulRoute.index_set():
                    ret_val += 0.5 * (model.FlowStrainMulRoute[indx] - model.FlowStrainMulRouteCenter[indx]) ** 2
            return ret_val            
        self.prox_max_model = cp.deepcopy(cmodel)
        for v in self.prox_max_model.component_objects(pyo.Var, active = True):
            for indx in v:
                v[indx].unfix()
        self.prox_max_model.ObjSmoothDual.deactivate()
        self.prox_max_model.ObjProxMax = pyo.Objective(rule = ProximalRule, sense = pyo.maximize)
        self.prox_max_model.ObjProxMax.activate()
        solver = sm.AslBaronSolver()
        result = solver.Solve(self.prox_max_model, False)
        self.R2 = None
        if result is not None:
            self.R2 = pyo.value(self.prox_max_model.ObjProxMax)

        #archive of the lb solutions
        self.f_ref = 0
        self.f_ref_data = []
        self.best_cmodel_feasible = None

        #init parameters
        self.Lmu = 0.0        
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
                self.M += pyo.value(relaxed_constraint_expr_rhs(cmodel, *indx)) ** 2
                self.B += pyo.value(relaxed_constraint_expr_lhs(cmodel, *indx)) ** 2
                self.lm_initial.append(pyo.value(LagrangianMultipliers[indx]))
        self.dprev = [0] * len(self.lm_initial)
        #change parameter for faste convergence
        self.M = 0.0
        self.B = 1.0

    def RetrieveBest(self):
        #retrieve best feasible if exist
        if self.best_cmodel_feasible is None:
            return super().RetrieveBest()
        return self.best_cmodel_feasible

    def UpdateMultipliers(self, cmodel, master_solver):
        #lower bound
        if self.optimal_obj is None:
            if (self.n_iter - 1) % 10 == 0:
                solution = sm.isolver.ISolver.ExtractSolution(cmodel)
                routes = solution[2]
                rsm = mg.mgenerator.RsModel()
                rsm.amodel = self.amodel_master
                rsm.init_data = self.init_data_master
                solution_recovered = mp.RecoverFeasibleStrain(rsm, routes, sm.CplexSolver())
                f_ref_rec = solution_recovered['Objective']
                self.rec_cmodel = solution_recovered['Cmodel']
                self.f_ref_data.append(f_ref_rec)
                self.f_ref = min(self.f_ref_data)
                if self.f_ref == f_ref_rec:
                    self.best_cmodel_feasible = cp.deepcopy(cmodel)
        else:
            self.f_ref = self.optimal_obj
        
        #dynamic smoothness
        epsilon_error = 1e-3
        smoothness_updated = max( self.f_ref - self.best_solution[0], epsilon_error * abs(self.f_ref) ) / (2 * self.R2)
        cmodel.Smoothness = smoothness_updated

        #iteration params
        k = self.n_iter
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

        #Lagrange multiplier calculate
        self.Lmu = self.M + self.B / cmodel.Smoothness.value
        lm_updated = []
        for indx, (lm_value, sign) in enumerate(self.lm):
            pi = lm_value + self.gradient[indx] / self.Lmu
            dzeta = self.lm_initial[indx] + delta[k - 1] *  self.dprev[indx] / self.Lmu
            d = iota[k] * self.gradient[indx] + (1 - iota[k]) * self.dprev[indx]
            self.dprev[indx] = d
            if sign == '<=':
                pi = max(0, pi)
                dzeta = max(0, dzeta)
            lm_updated_value = iota[k + 1] * dzeta + (1 - iota[k + 1]) * pi
            lm_updated.append(lm_updated_value)

        #Lagrange multiplier update
        indx_total = 0
        for names in self.relaxation_names:
            relaxed_set = getattr(cmodel, names[1])
            LagrangianMultipliers = getattr(cmodel, names[2])
            for indx in relaxed_set:
                LagrangianMultipliers[indx] = lm_updated[indx_total]
                self.lagr_mult_stop[indx_total].PutValue(pyo.value(LagrangianMultipliers[indx]))
                indx_total += 1

        #Update proximal parameters#
        if hasattr(cmodel, 'FlowStrain'):
            for indx in cmodel.FlowStrain.index_set():
                cmodel.FlowStrainCenter[indx] = cmodel.FlowStrain[indx].value
        if hasattr(cmodel, 'FlowRoute'):
            for indx in cmodel.FlowRoute.index_set():
                cmodel.FlowRouteCenter[indx] = cmodel.FlowRoute[indx].value
        if hasattr(cmodel, 'FlowStrainMulRoute'):
            for indx in cmodel.FlowStrainMulRoute.index_set():
                cmodel.FlowStrainMulRouteCenter[indx] = cmodel.FlowStrainMulRoute[indx].value

    def CheckExit(self):        
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        var_stop = sum([int(sc.CheckStop()) for sc in self.var_stop_crit]) == len(self.var_stop_crit)
        if lm_stop or var_stop:
            return True