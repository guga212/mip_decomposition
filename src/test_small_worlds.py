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
network = net.NetworkGraph.GenerateSmallWorld(world_init_data, 0, 2)
network.GenerateRandomSrcDst(3)

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
dec.SubnetsDecomposer(rs_model_dec, cr.CoordinatorGradient(step_rule=cr.gradstep.DiminishingFractionRule(0.1, 20)),
                        network.GetWorldNodes())

#initialize solvers
opt_couenne = sm.minlpsolvers.CouenneSolver()

#solve
solution = opt_couenne.Solve(rs_model.cmodel)
objective, strains, routes = ( solution['Objective'], solution['Strain'], solution['Route'] )

#validate constraints violations
viol = mp.FindConstraintsViolation(rs_model.cmodel)

#put solution of the problem into the network graph
path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
flow_list_sol = strains
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
net.PlotNetwork(network, 'Original formulation')

net.drawer.ShowAll()
debug_stop = 1