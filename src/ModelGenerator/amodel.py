import pyomo.environ as pyo

class ARoutingStrainModel():
    def __init__(self):
        self.model = pyo.AbstractModel()

        self.model.Flows = pyo.Set()
        self.model.Arcs = pyo.Set(dimen = 2)
        self.model.Nodes = pyo.Set()

        self.model.Src = pyo.Param(self.model.Flows, within = pyo.NonNegativeIntegers)
        self.model.Dst = pyo.Param(self.model.Flows, within = pyo.NonNegativeIntegers)
       
        def NodesOut_init(model, node):
            retval = []
            for (i,j) in model.Arcs:
                if i == node:
                    retval.append(j)
            return retval
        self.model.NodesOut = pyo.Set(self.model.Nodes, initialize = NodesOut_init)        
        def NodesIn_init(model, node):
            retval = []
            for (i,j) in model.Arcs:
                if j == node:
                    retval.append(i)
            return retval
        self.model.NodesIn = pyo.Set(self.model.Nodes, initialize = NodesIn_init) 

        self.model.Capacity = pyo.Param(self.model.Arcs, within = pyo.NonNegativeReals)

        self.model.FlowLb =  pyo.Param(within=pyo.NonNegativeIntegers)
        self.model.FlowUb =  pyo.Param(within=pyo.NonNegativeIntegers) 

        self.model.FlowStrain = pyo.Var(self.model.Flows, domain = pyo.NonNegativeReals)
        self.model.FlowRoute = pyo.Var(self.model.Flows, self.model.Arcs, domain = pyo.Binary)

        def ConstraintBound(model, flow):
            return (model.FlowLb, model.FlowStrain[flow], model.FlowUb)
        self.model.ConstrBound = pyo.Constraint(self.model.Flows, rule = ConstraintBound)

        

