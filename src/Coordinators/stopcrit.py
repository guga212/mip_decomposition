import collections


class StopCriterion:
    def __init__(self, slope_req, samples_nmb, excess_nmb_max = 1, 
                    slope_compare_function = (lambda slope, slope_req: slope >= slope_req) ):
        self.slope_req = slope_req
        self.values_arch = collections.deque([], samples_nmb)
        self.excess_nmb = 0
        self.excess_nmb_max = excess_nmb_max
        self.slope_compare_function = slope_compare_function
    def Reset(self):
        self.values_arch.clear()
        self.excess_nmb = 0
    def CalculateCriterion(self):
        if len(self.values_arch) ==  self.values_arch.maxlen:
            slope = (self.values_arch[0] - self.values_arch[-1]) / self.values_arch.maxlen
            if self.slope_compare_function(slope, self.slope_req) == False:
                self.excess_nmb += 1
                self.values_arch.clear()
            else:
                self.excess_nmb = 0
    def PutValue(self, value):
        self.values_arch.appendleft(value)
        self.CalculateCriterion()
    def CheckStop(self):
        return self.excess_nmb >= self.excess_nmb_max