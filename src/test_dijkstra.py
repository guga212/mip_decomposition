import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import ModelProcessors as mp

#define flow bounds
FLOW_BOUNDS = (0.1, 3)
CAPACITY_BOUNDS = (0.2, 2)

#random network
net.NetworkGraph.SetRandomSeed(228)
network = net.NetworkGraph.GenerateRandom(10, 38, *CAPACITY_BOUNDS)
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
rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, FLOW_BOUNDS)
rs_model_heur = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, FLOW_BOUNDS)

#initialize solvers
opt_heur = sm.DHeuristicSolver('List')
#opt = sm.minlpsolvers.CouenneSolver()
#opt = sm.milpsolvers.GlpkSolver()
opt = sm.miqppsolver.CplexSolver()

#solve
solution_orig = opt.Solve(rs_model.cmodel)
objective_orig, strains_orig, routes_orig, time_orig = ( solution_orig['Objective'], solution_orig['Strain'], solution_orig['Route'], solution_orig['Time'] )
solution_heur = opt_heur.Solve(rs_model_heur.cmodel)
objective_heur, strains_heur, routes_heur, time_heur = ( solution_heur['Objective'], solution_heur['Strain'], solution_heur['Route'], solution_heur['Time'] )

#recover feasible for given routes
solution_rec = mp.RecoverFeasibleStrain(rs_model, routes_heur, opt)
objective_rec, strains_rec, routes_rec, rec_cmodel, time_rec = ( solution_rec['Objective'], solution_rec['Strain'], solution_rec['Route'], solution_rec['Cmodel'], solution_rec['Time'])

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
print(f'Recovery:\nObjective: {objective_rec}, Time: {time_rec} ')
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