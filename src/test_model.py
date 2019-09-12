import ModelGenerator as mg

import NetworkFlow as net
import SolverManager as sm

#random network
net.NetworkGraph.SetRandomSeed(231)
network = net.NetworkGraph.GenerateRandom(8, 28, 0, 2)
network.GenerateRandomSrcDst(3)

#full network
# network = net.NetworkGraph.GenerateFull(8, 1.6)
# network.SetSrcDst((0,3), (2, 3), (1, 5))

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create two model generators for two formulation
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
nmg_alt = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.ReformulatedConstraintsGenerator())

#create model instance from the generator
rsm_base_instance = nmg.CreateInstance(f_list, sd_dict, n_list, a_list, c_dict, (0,3))
rsm_base_instance_alt = nmg_alt.CreateInstance(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#solve problem
opt_glpk = sm.milpsolvers.GlpkSolver()
objective, strains, routes, solver_output = opt_glpk.Solve(rsm_base_instance)
objective_alt, strains_alt, routes_alt, solver_output_alt = opt_glpk.Solve(rsm_base_instance_alt)

#validate constraints violations
viol = mg.FindConstraintsViolation(rsm_base_instance)
viol_alt = mg.FindConstraintsViolation(rsm_base_instance_alt)

#compare solutions
eq1 = (objective == objective_alt)
eq2 = (strains == strains_alt)
eq3 = (routes == routes_alt)

#put solution of the problem into the network graph
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
flow_list_sol = strains
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network)

#put solution of the problem's alternative formulation into the network graph
path_list_sol_alt =  [ [edge for edge, value in route.items() if value == 1] for route in routes_alt ]
flow_list_sol_alt = strains_alt
network.SetPath(*path_list_sol_alt)
network.SetFlows(*flow_list_sol_alt)
net.PlotNetwork(network)

debug_val = 1