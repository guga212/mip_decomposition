import math
import pyomo.environ as pyo

class IStepRule:
    def __init__(self, *args):
        pass
    def Reset(self):
        pass
    def UpdateData(self, **kwargs):
        self.dual_val = kwargs['obj']
        self.gradient = kwargs['gradient']
        self.variables = kwargs['variables']
    def GetVariables(self):
        ret_val = []
        step = self.GetStep()
        for indx, data in enumerate(self.variables):
            v,s = data
            z_val = v + step * self.gradient[indx]
            if s == '<=':
                z_val = max(0, z_val)
            ret_val.append(z_val)
        return ret_val
    def GetStep(self):
        return 0

class ConstantStepRule(IStepRule):
    def __init__(self, step):
        self.step = step
    def GetStep(self):
        return self.step

class SquareSummableStepRule(IStepRule):
    def __init__(self, nom, den):
        self.nom = nom
        self.den = den
        self.n_iter = 0
    def Reset(self):
        self.n_iter = 0
    def GetStep(self):
        self.n_iter += 1
        self.step = self.nom / (self.den + self.n_iter)
        return self.step

class DiminishingStepRule(IStepRule):
    def __init__(self, nom):
        self.nom = nom
        self.n_iter = 0
    def Reset(self):
        self.n_iter = 0
    def GetStep(self):
        self.n_iter += 1
        self.step = self.nom / math.sqrt(self.n_iter)
        return self.step

class OptimalObjectiveStepRule(IStepRule):
    def __init__(self, scale):
        self.n_iter = 0
        self.gradient_norm = 0
        self.scale = scale
    def UpdateData(self, **kwargs):
        super().UpdateData(**kwargs)
        self.gradient_norm = sum([ gr**2 for gr in self.gradient])
    def Reset(self):
        self.n_iter = 0
        self.gradient_norm = 0
    def SetUb(self, dual_ub):
        self.dual_ub = dual_ub
    def GetStep(self):
        self.n_iter += 1
        self.step = self.scale * (self.dual_ub - self.dual_val) / self.gradient_norm
        return self.step

class ObjectiveLevelStepRule(OptimalObjectiveStepRule):
    def __init__(self, R, scale = 1.0):
        self.f_lev = {}
        self.f_rec = { 0: float('-inf') }
        self.x_rec = {}
        self.grad_rec = {}
        self.R = R
        self.delta = { 1: 1.0 }
        self.sigma = { 1: 0.0 }
        self.k = 0
        self.K = {1 : 1}
        self.l = 1
        super().__init__(scale)

    def InitializeLevelStep(self):
        self.delta[1] = math.sqrt(self.gradient_norm) * self.R
        self.k += 1
    
    def IterateLevelStep(self):
        #step 1 : objective evaluation
        if self.dual_val > self.f_rec[self.k - 1]:
            self.f_rec[self.k] = self.dual_val
            self.x_rec[self.k] = self.variables
            self.grad_rec[self.k] = self.gradient
        else:
            self.f_rec[self.k] = self.f_rec[self.k - 1]
            self.x_rec[self.k] = self.x_rec[self.k - 1]
            self.grad_rec[self.k] = self.grad_rec[self.k - 1]

        #setp 2 : stopping criterion
        if self.gradient_norm == 0:
            return None

        #step 3 : sufficient descent detection
        if (self.dual_val - self.f_rec[self.K[self.l]]) >= 0.5 * self.delta[self.l]:
            self.K[self.l + 1] = self.k
            self.sigma[self.k] = 0
            self.delta[self.l + 1] = self.delta[self.l]
            self.l += 1
        else:
            #step 4 : oscilation detection
            if self.sigma[self.k] > self.R:
                self.K[self.l + 1] = self.k
                self.sigma[self.k] = 0
                self.delta[self.l + 1] = 0.5 * self.delta[self.l]
                self.variables = self.x_rec[self.k]
                self.gradient = self.grad_rec[self.k]
                self.l = self.l + 1
        
        #step 5 : projections
        self.f_lev[self.k] = self.f_rec[self.K[self.l]] + self.delta[self.l]
        self.z = []
        self.step = self.scale * (self.f_lev[self.k] - self.dual_val) / self.gradient_norm

        for indx, data in enumerate(self.variables):
            v,s = data
            z_val = v +  self.step * self.gradient[indx]
            if s == '<=':
                z_val = max(0, z_val)
            self.z.append(z_val)        

        #step 6 : path update
        self.sigma[self.k + 1] = self.sigma[self.k] + math.sqrt( sum( [ (z - self.variables[indx][0])**2 for indx, z in enumerate(self.z) ] ) )
        self.k += 1

    def GetVariables(self):
        if self.k == 0:
            self.InitializeLevelStep()
        self.IterateLevelStep()
        return self.z