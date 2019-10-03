from .coordgrad import CoordinatorGradient
from .gradstep import IStepRule
import pyomo.environ as pyo
import copy as cp

class CoordinatorSurrogateGradient(CoordinatorGradient):
    def __init__(self, step_rule = IStepRule()):
        super().__init__(step_rule)
    def GenerateSolvingPolciy(self, local_nmb):
        def SolvingPolicy(solve):
            if solve(self.n_iter % local_nmb) == False:
                return False
        self.solving_policy = SolvingPolicy
        return self.solving_policy

class CoordinatorFsaGradient(CoordinatorGradient):
    def __init__(self, step_rule = IStepRule()):
        super().__init__(step_rule)
    def UpdateMultipliers(self, cmodel):
        super().UpdateMultipliers(cmodel)
