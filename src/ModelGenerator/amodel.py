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

        def ConstraintRoute(model, flow, node):
            sum_eq = - 1 if node == model.Src[flow] else + 1 if node == model.Dst[flow] else 0
            return 0 \
            + sum(model.FlowRoute[flow, i, node] for i in model.NodesIn[node]) \
            - sum(model.FlowRoute[flow, node, j] for j in model.NodesOut[node]) \
            == sum_eq
        self.model.ConstrRoute = pyo.Constraint(self.model.Flows, self.model.Nodes, rule = ConstraintRoute)

        #######nonlinear constraints
        # def ConstraintCapacity(model, arc_s, arc_d):
        #     return model.Capacity[arc_s, arc_d] \
        #     >= sum(model.FlowStrain[flow] * model.FlowRoute[flow, arc_s, arc_d] for flow in model.Flows)
        # self.model.ConstrCapacity = pyo.Constraint(self.model.Arcs, rule = ConstraintCapacity)

        self.model.FlowStrainMulRoute = pyo.Var(self.model.Flows, self.model.Arcs, domain = pyo.NonNegativeReals)        
        def FlowStrainMulRouteConstraint1(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] - model.FlowStrain[flow] \
            >= 1.1*model.FlowUb*(model.FlowRoute[flow, node_s, node_d] - 1)
        self.model.FlowStrainMulRouteConstraint1 = pyo.Constraint(self.model.Flows, self.model.Arcs, rule = FlowStrainMulRouteConstraint1)
        def FlowStrainMulRouteConstraint2(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            >= 0
        self.model.FlowStrainMulRouteConstraint2 = pyo.Constraint(self.model.Flows, self.model.Arcs, rule = FlowStrainMulRouteConstraint2)
        def FlowStrainMulRouteConstraint3(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            <= model.FlowStrain[flow]
        self.model.FlowStrainMulRouteConstraint3 = pyo.Constraint(self.model.Flows, self.model.Arcs, rule = FlowStrainMulRouteConstraint3)
        def FlowStrainMulRouteConstraint4(model, flow, node_s, node_d):
            return model.FlowStrainMulRoute[flow, node_s, node_d] \
            <= 1.1*model.FlowUb*model.FlowRoute[flow, node_s, node_d]
        self.model.FlowStrainMulRouteConstraint4 = pyo.Constraint(self.model.Flows, self.model.Arcs, rule = FlowStrainMulRouteConstraint4)
        def ConstraintCapacity(model, node_s, node_d):
            return sum(model.FlowStrainMulRoute[flow, node_s, node_d] for flow in model.Flows) \
            <= model.Capacity[node_s, node_d]
        self.model.ConstrCapacity = pyo.Constraint(self.model.Arcs, rule = ConstraintCapacity)
        

        # def ObjectiveQuad(model):
        #     return 0 + \
        #     sum((-1/model.FlowUb) * model.FlowStrain[flow] ** 2 + 2 * model.FlowStrain[flow] for flow in model.Flows) + \
        #     sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)
        def ObjectiveLin(model):
            return \
            sum((1/model.FlowUb) * (model.FlowStrain[flow] - model.FlowLb) for flow in model.Flows) + \
            -0.05*sum(model.FlowRoute[flow, arc] for flow in model.Flows for arc in model.Arcs)


        self.model.Obj = pyo.Objective(rule = ObjectiveLin, sense = pyo.maximize)

