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
opt_solver = sm.milpsolvers.GlpkSolver()
#opt_solver = sm.miqppsolver.CplexSolver()
#opt_solver = sm.minlpsolvers.CouenneSolver()

#solve without decomposition
solution_orig = opt_solver.Solve(rs_model.cmodel)
objective_orig, strains_orig, routes_orig, time_orig = ( solution_orig['Objective'], solution_orig['Strain'], solution_orig['Route'], solution_orig['Time'] )

#create flow decomposer
#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorGradient(step_rule=cr.gradstep.ConstantStepRule(0.1)), 'CapacityConstraintLinear')
#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorGradient(step_rule=cr.gradstep.SquareSummableStepRule(0.8, 20)), 'CapacityConstraintLinear')
#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorSurrogateGradient(step_rule=cr.gradstep.ConstantStepRule(0.05)), 'CapacityConstraintLinear')
#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorSurrogateGradient(step_rule=cr.gradstep.SquareSummableStepRule(0.08, 10)), 'CapacityConstraintLinear')
#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorSurrogateGradient(step_rule=cr.gradstep.DiminishingStepRule(0.03)), 'CapacityConstraintLinear')
flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorFsaGradient(step_rule=cr.gradstep.DiminishingStepRule(0.03)), 'CapacityConstraintLinear')

#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorGradient(step_rule = cr.gradstep.ObjectiveLevelStepRule(0.76, 1.0)), 'CapacityConstraintLinear')
#flow_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorSurrogateGradient(step_rule = cr.gradstep.ObjectiveLevelStepRule(0.8)), 'CapacityConstraintLinear')

#solve decomposed
solution_dec = flow_dec.Solve( opt_solver, [opt_solver for _ in f_list] )
objective_dec, objective_dual_dec, strains_dec, routes_dec, time_dec = ( solution_dec['Objective'], solution_dec['ObjectiveDual'], 
                                                                        solution_dec['Strain'], solution_dec['Route'], solution_dec['Time'] )

#recover feasible for given routes
soultion_rec = mp.RecoverFeasibleStrain(rs_model, routes_dec, opt_solver)
objective_rec, strains_rec, routes_rec, rec_cmodel = ( soultion_rec['Objective'], soultion_rec['Strain'], soultion_rec['Route'], soultion_rec['Cmodel'])

#validate constraints violations
viol_orig = mp.FindConstraintsViolation(rs_model.cmodel, strains_orig, routes_orig)
viol_dec = mp.FindConstraintsViolation(rs_model.cmodel, strains_dec, routes_dec)
viol_rec = mp.FindConstraintsViolation(rs_model.cmodel, strains_rec, routes_rec)

#print results
print('')
print('###################!RESULTS!#############################')
print(f'Original:\nObjective: {objective_orig}, Time: {time_orig}')
print('__________________________________________________________')
print(f'Decomposition:\nObjective: {objective_dec}, ObjectiveDual: {objective_dual_dec}, Time: {time_dec}')
print('__________________________________________________________')
print(f'Recovery:\nObjective: {objective_rec}')
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
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_dec ]
flow_list_sol = strains_dec
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST DECOMPOSER')

#draw network decomposed recovered
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_rec ]
flow_list_sol = strains_rec
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'TEST RECOVERED')

#show all plots
net.drawer.ShowAll()

debug_stop = 1