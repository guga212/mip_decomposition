import pyomo.environ as pyo

class ARoutingStrainModel(pyo.AbstractModel):
    def __init__(self):

        super().__init__()

        self.Flows = pyo.Set()
        self.Arcs = pyo.Set(dimen = 2)
        self.Nodes = pyo.Set()

        self.Src = pyo.Param(self.Flows, within = pyo.NonNegativeIntegers)
        self.Dst = pyo.Param(self.Flows, within = pyo.NonNegativeIntegers)
       
        def NodesOut_init(model, node):
            retval = []
            for (i,j) in model.Arcs:
                if i == node:
                    retval.append(j)
            return retval
        self.NodesOut = pyo.Set(self.Nodes, initialize = NodesOut_init)        
        def NodesIn_init(model, node):
            retval = []
            for (i,j) in model.Arcs:
                if j == node:
                    retval.append(i)
            return retval
        self.NodesIn = pyo.Set(self.Nodes, initialize = NodesIn_init) 

        self.Capacity = pyo.Param(self.Arcs, within = pyo.NonNegativeReals)

        self.FlowLb =  pyo.Param(within=pyo.NonNegativeIntegers)
        self.FlowUb =  pyo.Param(within=pyo.NonNegativeIntegers) 

        self.FlowStrain = pyo.Var(self.Flows, domain = pyo.NonNegativeReals)
        self.FlowRoute = pyo.Var(self.Flows, self.Arcs, domain = pyo.Binary)

        def ConstraintBound(model, flow):
            return (model.FlowLb, model.FlowStrain[flow], model.FlowUb)
        self.ConstraintBound = pyo.Constraint(self.Flows, rule = ConstraintBound)

        self.Suffix = pyo.Suffix(datatype = None)
                        

