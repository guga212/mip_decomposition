import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import Decomposers as dec

#random network
net.NetworkGraph.SetRandomSeed(666)
network = net.NetworkGraph.GenerateRandom(6, 12, 0, 2)
network.GenerateRandomSrcDst(2)

#get network params
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

#create model generator and get model instace from it
nmg = mg.RsModelGenerator(mg.LinearObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
rs_model = nmg.CreateModel(f_list, sd_dict, n_list, a_list, c_dict, (0,3))

#create flow decomposer
flow_dec = dec.FlowDecomposition(rs_model)