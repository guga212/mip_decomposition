import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import Decomposers as dec
import Coordinators as cr

#random network
net.NetworkGraph.SetRandomSeed(666)
network = net.NetworkGraph.GenerateRandom(6, 12, 0, 2)
network.GenerateRandomSrcDst(2)

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create model generator and get model instace from it
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
rs_model = nmg.CreateModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#initialize solvers
opt_glpk = sm.milpsolvers.GlpkSolver()

#create flow decomposer
flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorGradient)
decompose_solution = flow_dec.Solve(opt_glpk)
objective, strains, routes, time = ( decompose_solution['Objective'], decompose_solution['Strain'], 
                                        decompose_solution['Route'], decompose_solution['Time'] )

#validate constraints violations
viol = mg.FindConstraintsViolation(flow_dec.cmodel)

#TEST
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
flow_list_sol = strains
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST DECOMPOSER')
net.drawer.ShowAll()