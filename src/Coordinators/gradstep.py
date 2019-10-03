import pyomo.environ as pyo

class IStepRule:
    def __init__(self, *args):
        self.step = 0        
    def GetStep(self, cmodel):
        return self.step
    def Reset(self):
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