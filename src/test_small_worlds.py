import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import ModelProcessors as mp
import Decomposers as dec
import Coordinators as cr

#random network
net.NetworkGraph.SetRandomSeed(231)
world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 6,  'ExternalEdgesNumber': 3 }, 
                    {'NodesNumber': 8, 'EdgesNumber': 16, 'ExternalEdgesNumber': 3 },
                    {'NodesNumber': 4, 'EdgesNumber': 8,  'ExternalEdgesNumber': 2 }
                    ]  
network = net.NetworkGraph.GenerateSmallWorld(world_init_data, 0.5, 3)
network.GenerateRandomSrcDst(2)

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create model generators for two formulation
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())

#create models from the generator
rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))
rs_model_dec = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#subnet decomposition init
# subnet_dec = dec.SubnetsDecomposer(rs_model_dec, cr.CoordinatorGradient(step_rule=cr.gradstep.DiminishingFractionRule(0.1, 20)),
#                                     network.GetWorldNodes())
subnet_dec = dec.SubnetsDecomposer(rs_model_dec, cr.CoordinatorFsaGradient(step_rule=cr.gradstep.DiminishingFractionRule(0.1, 20)),
                                    network.GetWorldNodes())

#initialize solvers
opt_solver = sm.milpsolvers.GlpkSolver()
#opt_solver = sm.minlpsolvers.CouenneSolver()

#solve
solution_orig = opt_solver.Solve(rs_model.cmodel)
objective_orig, strains_orig, routes_orig, time_orig = ( solution_orig['Objective'], solution_orig['Strain'], solution_orig['Route'], solution_orig['Time'] )

#solve decomposed
solution_dec = subnet_dec.Solve( opt_solver, [opt_solver for _ in network.GetWorldNodes()] )
objective_dec, objective_dual_dec, strains_dec, routes_dec, time_dec = ( solution_dec['Objective'], solution_dec['ObjectiveDual'], 
                                                                        solution_dec['Strain'], solution_dec['Route'], solution_dec['Time'] )

#validate constraints violations
viol = mp.FindConstraintsViolation(rs_model.cmodel)
#viol_dec = mp.FindConstraintsViolation(subnet_dec.cmodel)

#print results
print('')
print('###################!RESULTS!#############################')
print(f'Original:\nObjective: {objective_orig}, Time: {time_orig}')
print('__________________________________________________________')
print(f'Decomposition:\nObjective: {objective_dec}, ObjectiveDual: {objective_dual_dec}, Time: {time_dec}')
print('__________________________________________________________')
print('#####################!END!###############################')
print('')

#draw network original
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_orig ]
flow_list_sol = strains_orig
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST ORIGINAL')

#draw network decomposed
path_list_sol =  [ [(indx[1], indx[2]) for indx, value in route.items() if value == 1] for route in routes_dec ]
flow_list_sol =  [ min([flow_subnet for flow_subnet in strains_dec[flow_key]]) for flow_key in strains_dec ]
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST DECOMPOSER')

net.drawer.ShowAll()
debug_stop = 1