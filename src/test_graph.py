import NetworkFlow as net

# create network
random_network = net.NetworkGraph.GenerateRandom(8, 64, 0, 5)

# modify network
random_network.SetSrcDst((2,3),(3, 4))
random_network.GenerateRandomSrcDst(2)    

# test graph getter funcitions
n_list = random_network.GetNodeList()
f_list = random_network.GetFlowList()
sd_dict = random_network.GetSrcDstDict()
a_list = random_network.GetArcList()
c_dict = random_network.GetCapacityParam()

# draw network
network_drawer = net.NetworkDrawer(random_network)

# show network
network_drawer.Show()

print("exit 0")
