import ModelGenerator as mg
import ModelProcessors as mp
import NetworkFlow as net
import SolverManager as sm
import Decomposers as dec
import Coordinators as cr
import Drawer as dr

#define flow bounds
FLOW_BOUNDS = (0.1, 3)
CAPACITY_BOUNDS = (0.2, 2)

#random network
net.NetworkGraph.SetRandomSeed(666)
network = net.NetworkGraph.GenerateRandom(6, 18, *CAPACITY_BOUNDS)
network.GenerateRandomSrcDst(6)

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create model generator and get model instace from it
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, FLOW_BOUNDS)

#initialize solvers
#opt_solver = sm.milpsolvers.GlpkSolver()
opt_solver = sm.miqppsolver.CplexSolver()
#opt_solver = sm.minlpsolvers.CouenneSolver()

#solve without decomposition
solution_orig = opt_solver.Solve(rs_model.cmodel)
objective_orig, strains_orig, routes_orig, time_orig = ( solution_orig['Objective'], solution_orig['Strain'], solution_orig['Route'], solution_orig['Time'] )

#create flow decomposer
model_dec = dec.FlowDecomposer(rs_model, cr.CoordinatorFastGradient() , 'CapacityConstraintLinear')

#solve decomposed
local_solvers = [opt_solver for _ in f_list]
solution_dec = model_dec.Solve( opt_solver,  local_solvers)
objective_dec, objective_dual_dec, strains_dec, routes_dec, time_dec = ( solution_dec['Objective'], solution_dec['ObjectiveDual'], 
                                                                        solution_dec['Strain'], solution_dec['Route'], solution_dec['Time'] )

#show decomposition iterating data
data = model_dec.RecordedData
dr.PlotIterationData('Objective', data['MasterObj'])
for name, data in data['Multipliers'].items():
    dr.PlotIterationData(name, data)

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
dr.PlotNetwork(network, 'TEST ORIGINAL')

#draw network decomposed
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_dec ]
flow_list_sol = strains_dec
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
dr.PlotNetwork(network, 'TEST DECOMPOSER')

#draw network decomposed recovered
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes_rec ]
flow_list_sol = strains_rec
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
dr.PlotNetwork(network, 'TEST RECOVERED')

#show all plots
dr.ShowAll()

debug_stop = 1