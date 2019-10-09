import pyomo.environ as pyo

class IStepRule:
    def __init__(self, *args):
        self.step = 0        
    def GetStep(self, cmodel):
        return self.step
    def Reset(self):
        pass
    def PutGradient(self, gradient):
        pass

class ConstantStepRule(IStepRule):
    def __init__(self, step):
        self.step = step
    def GetStep(self, cmodel):
        return self.step

class DiminishingFractionRule(IStepRule):
    def __init__(self, nom, den):
        self.nom = nom
        self.den = den
        self.n_iter = 0
    def GetStep(self, cmodel):
        self.n_iter += 1
        self.step = self.nom / (self.den + self.n_iter)
        return self.step
    def Reset(self):
        self.n_iter = 0

class OptimalObjectiveStepRule(IStepRule):
    def __init__(self, decrease_param = 1.0):
        self.decrease = decrease_param
        self.n_iter = 0
        self.gradient_norm = 0
        self.dual_ub = 0
    def PutGradient(self, gradient):
        self.gradient_norm = sum([ gr**2 for gr in gradient])
    def GetStep(self, cmodel):
        self.n_iter += 1
        self.gamma = (1 + self.decrease) / ( self.n_iter + self.decrease )
        self.dual_val = pyo.value(cmodel.ObjDual)
        self.step = self.gamma * (self.dual_ub - self.dual_val) / self.gradient_norm
        return self.step
    def Reset(self):
        self.n_iter = 0