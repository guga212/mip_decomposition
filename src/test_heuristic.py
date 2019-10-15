import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import ModelProcessors as mp

#random network
net.NetworkGraph.SetRandomSeed(228)
network = net.NetworkGraph.GenerateRandom(10, 38, 0, 2)
network.GenerateRandomSrcDst(6)

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create model generator
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())

#create model from the generator
rsm_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))
rsm_model_heur = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#initialize solvers
opt_heur = sm.HeuristicSolver()
opt = sm.milpsolvers.GlpkSolver()

#solve
solution = opt.Solve(rsm_model.cmodel)
objective, strains, routes = ( solution['Objective'], solution['Strain'], solution['Route'] )
solution_heur = opt_heur.Solve(rsm_model_heur.cmodel)
objective_heur, strains_heur, routes_heur = ( solution_heur['Objective'], solution_heur['Strain'], solution_heur['Route'] )


#validate constraints violations
viol = mp.FindConstraintsViolation(rsm_model.cmodel)
viol_heur = mp.FindConstraintsViolation(rsm_model_heur.cmodel)


#put solution of the problem into the network graph
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
flow_list_sol = strains
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'Classic solver')

#put solution_heur of the problem into the network graph
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_heur ]
flow_list_sol = strains_heur
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'Heuristic solver')


net.drawer.ShowAll()