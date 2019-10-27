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
nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())

#create model from the generator
rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))
rs_model_heur = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#initialize solvers
opt_heur = sm.HeuristicSolver('List')
opt = sm.miqppsolver.CplexSolver()

#solve
solution_orig = opt.Solve(rs_model.cmodel)
objective_orig, strains_orig, routes_orig, time_orig = ( solution_orig['Objective'], solution_orig['Strain'], solution_orig['Route'], solution_orig['Time'] )
solution_heur = opt_heur.Solve(rs_model_heur.cmodel)
objective_heur, strains_heur, routes_heur, time_heur = ( solution_heur['Objective'], solution_heur['Strain'], solution_heur['Route'], solution_heur['Time'] )

#recover feasible for given routes
soultion_rec = mp.RecoverFeasibleStrain(rs_model, routes_heur, opt)
objective_rec, strains_rec, routes_rec, rec_cmodel = ( soultion_rec['Objective'], soultion_rec['Strain'], soultion_rec['Route'], soultion_rec['Cmodel'])

#validate constraints violations
viol = mp.FindConstraintsViolation(rs_model.cmodel, strains_orig, routes_orig)
viol_heur = mp.FindConstraintsViolation(rs_model.cmodel, strains_heur, routes_heur)

#print results
print('')
print('###################!RESULTS!#############################')
print(f'Original:\nObjective: {objective_orig}, Time: {time_orig}')
print('__________________________________________________________')
print(f'Heuristic:\nObjective: {objective_heur}, Time: {time_heur}')
print('__________________________________________________________')
print(f'Recovery:\nObjective: {objective_rec}')
print('__________________________________________________________')
print('#####################!END!###############################')
print('')


#put solution of the problem into the network graph
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_orig ]
flow_list_sol = strains_orig
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