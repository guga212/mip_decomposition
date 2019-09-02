import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm

#random network
network = net.NetworkGraph.GenerateRandom(8, 64, 0, 2)
network.GenerateRandomSrcDst(3)

#full network
# network = net.NetworkGraph.GenerateFull(8, 5)
# network.SetSrcDst((0,3),(5, 4))

#draw network
# network_drawer = net.NetworkDrawer(network)
# network_drawer.Show()


n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

nmg = mg.RsModelGenerator()
rsm_base_instance = nmg.CreateInstance(f_list, sd_dict, n_list, a_list, c_dict, (0,3))
print(f'Model instance: {rsm_base_instance.pprint()}')

opt_glpk = sm.milpsolvers.GlpkSolver()
objective, strains, routes, solver_output = opt_glpk.Solve(rsm_base_instance)
#print(f'Objective: {objective}')
#print(f'Strain values: {strains}')
#print(f'Routes values: {routes}')


path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
flow_list_sol = strains
network.SetPath(*path_list_sol)
network.SetFlows(*flow_list_sol)
network_drawer = net.NetworkDrawer(network)
network_drawer.Show()
