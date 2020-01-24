import NetworkFlow as net
import Drawer as dr

#Init seed
seed = 111
net.NetworkGraph.SetRandomSeed(seed)

CAPACITY_BOUNDS = (0.3, 1)


#Network example
world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                    {'NodesNumber': 12, 'EdgesNumber': 32, 'ExternalEdgesNumber': 3 }
                    ]
network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
network.GenerateRandomSrcDst(2)


#Network small
world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                      {'NodesNumber': 6, 'EdgesNumber': 4,  'ExternalEdgesNumber': 2 },
                      {'NodesNumber': 4, 'EdgesNumber': 6,  'ExternalEdgesNumber': 1 },
                      {'NodesNumber': 1, 'EdgesNumber': 3,  'ExternalEdgesNumber': 1 }
                    ]
network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
#network.GenerateRandomSrcDst(4)

# #Network large
# world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
#                     {'NodesNumber': 12, 'EdgesNumber': 32, 'ExternalEdgesNumber': 3 },
#                     {'NodesNumber': 6, 'EdgesNumber': 22,  'ExternalEdgesNumber': 2 },
#                      {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 2 }
#                     ]
# network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
#network.GenerateRandomSrcDst(12)

# #Network large
# world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
#                     {'NodesNumber': 12, 'EdgesNumber': 32, 'ExternalEdgesNumber': 3 },
#                     {'NodesNumber': 6, 'EdgesNumber': 22,  'ExternalEdgesNumber': 2 },
#                      {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 2 },
#                      {'NodesNumber': 9, 'EdgesNumber': 16,  'ExternalEdgesNumber': 1 },
#                      {'NodesNumber': 8, 'EdgesNumber': 20,  'ExternalEdgesNumber': 2 },
#                      {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 1 },
#                      {'NodesNumber': 3, 'EdgesNumber': 6,  'ExternalEdgesNumber': 1 },
#                     ]
# network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
# network.GenerateRandomSrcDst(32)

# test graph getter funcitions
n_list = network.GetNodeList()
f_list = network.GetFlowList()
sd_dict = network.GetSrcDstDict()
a_list = network.GetArcList()
c_dict = network.GetCapacityParam()

# plot network
dr.PlotNetwork(network, 'Test graph')

#show all drawn
dr.ShowAll()

