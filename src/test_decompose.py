import ModelGenerator as mg
import ModelProcessors as mp
import NetworkFlow as net
import SolverManager as sm
import Decomposers as dec
import Coordinators as cr

#random network
net.NetworkGraph.SetRandomSeed(666)
network = net.NetworkGraph.GenerateRandom(6, 18, 0, 2)
network.GenerateRandomSrcDst(6)

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create model generator and get model instace from it
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#initialize solvers
opt_glpk = sm.milpsolvers.GlpkSolver()

#solve without decomposition
soultion_orig = opt_glpk.Solve(rs_model.cmodel)
objective_orig, strains_orig, routes_orig = ( soultion_orig['Objective'], soultion_orig['Strain'], soultion_orig['Route'] )

#create flow decomposer
flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorGradient)
solution_dec = flow_dec.Solve( opt_glpk, [opt_glpk for _ in f_list] )
objective_dec, strains_dec, routes_dec, time_dec = ( solution_dec['Objective'], solution_dec['Strain'], 
                                        solution_dec['Route'], solution_dec['Time'] )

#recover feasible for given routes
soultion_rec = mp.RecoverFeasibleStrain(rs_model, routes_dec, opt_glpk)
objective_rec, strains_rec, routes_rec, rec_cmodel = ( soultion_rec['Objective'], soultion_rec['Strain'], soultion_rec['Route'], soultion_rec['Cmodel'])

#validate constraints violations
viol_orig = mp.FindConstraintsViolation(rs_model.cmodel)
viol_dec = mp.FindConstraintsViolation(flow_dec.cmodel)
viol_rec = mp.FindConstraintsViolation(rec_cmodel)

#draw network without decomposition
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_orig ]
flow_list_sol = strains_orig
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST ORIGINAL')

#draw network decomposed
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_dec ]
flow_list_sol = strains_dec
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST DECOMPOSER')

#draw network decomposed
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_rec ]
flow_list_sol = strains_rec
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST RECOVERED')

#show all plots
net.drawer.ShowAll()


debug_stop = 1