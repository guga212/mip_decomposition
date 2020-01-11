import BranchAndBound as bb
import NetworkFlow as net
import ModelGenerator as mg

#Init seed
seed = 111
net.NetworkGraph.SetRandomSeed(seed)

#define flow bounds
FLOW_BOUNDS = (0.001, 3)
CAPACITY_BOUNDS = (0.3, 1)

#Network medium
world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                     {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 2 }
                    ]
network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
network.GenerateRandomSrcDst(12)


bnb_solver = bb.BranchAndBoundSolver(network, FLOW_BOUNDS)
bnb_solver.Solve()