import pyomo.environ as pyo

class ARoutingStrainModel(pyo.AbstractModel):

    def InitializeBaseSets(self):
        self.Flows = pyo.Set()
        self.Nodes = pyo.Set()
        self.Arcs = pyo.Set(dimen = 2)

    def InitializeDerivedSets(self):
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

    def InitializeParameters(self):
        self.Src = pyo.Param(self.Flows, within = pyo.NonNegativeIntegers)
        self.Dst = pyo.Param(self.Flows, within = pyo.NonNegativeIntegers)       
        self.Capacity = pyo.Param(self.Arcs, within = pyo.NonNegativeReals)
        self.FlowLb = pyo.Param(within=pyo.NonNegativeReals, mutable = True)
        self.FlowUb = pyo.Param(within=pyo.NonNegativeReals, mutable = True)
        def StrainBoundsRule(model, *args):
            return ( model.FlowLb, model.FlowUb )
        self.StrainBoundsRule = StrainBoundsRule

    def InitializeVariables(self):
        self.FlowStrain = pyo.Var(self.Flows, domain = pyo.NonNegativeReals, bounds = self.StrainBoundsRule)
        self.FlowRoute = pyo.Var(self.Flows, self.Arcs, domain = pyo.Binary)

    def InitializeSuffix(self):
        self.Suffix = pyo.Suffix(datatype = None)

    def __init__(self):
        super().__init__()
        self.name = 'RoutingStrainAbstractModel'        
        self.InitializeBaseSets()
        self.InitializeDerivedSets()
        self.InitializeParameters()
        self.InitializeVariables()
        self.InitializeSuffix()
        


        
                        

