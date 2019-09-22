import NetworkFlow as net

# create network
random_network = net.NetworkGraph.GenerateRandom(8, 64, 0, 5)

# modify network
random_network.SetSrcDst((2,3),(3, 4))

# test graph getter funcitions
n_list = random_network.GetNodeList()
f_list = random_network.GetFlowList()
sd_dict = random_network.GetSrcDstDict()
a_list = random_network.GetArcList()
c_dict = random_network.GetCapacityParam()

# plot network
net.drawer.PlotNetwork(random_network, "Test graph")

#show all drawn
net.drawer.ShowAll()

