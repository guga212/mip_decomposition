from .coord import Coordinator
from .gradstep import IStepRule
from .stopcrit import StopCriterion
import pyomo.environ as pyo

class CoordinatorGradient(Coordinator):

    def __init__(self, step_rule = IStepRule()):
        super().__init__()
        self.step_rule = step_rule

    def UpdateIterationData(self, cmodel):
        super().UpdateIterationData(cmodel)
        #update step data
        self.step_rule.UpdateData(obj = self.obj_val, gradient = self.gradient, variables = self.lm)

    def UpdateMultipliers(self, cmodel, master_solver):
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