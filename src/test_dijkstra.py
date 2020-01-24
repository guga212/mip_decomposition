import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import ModelProcessors as mp
import Decomposers as dec
import Coordinators as cr
import TestFramework as tf


def RunTest(seed):

    #desribe run
    print('#######################################################')
    print(f'Seed is equal: {seed}')

    #define flow bounds
    FLOW_BOUNDS = (0.001, 3)
    CAPACITY_BOUNDS = (0.3, 1)

    #random network
    net.NetworkGraph.SetRandomSeed(seed)

    #Network small
    world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                          {'NodesNumber': 6, 'EdgesNumber': 4,  'ExternalEdgesNumber': 2 },
                          {'NodesNumber': 4, 'EdgesNumber': 6,  'ExternalEdgesNumber': 1 },
                          {'NodesNumber': 1, 'EdgesNumber': 3,  'ExternalEdgesNumber': 1 }
                        ]
    network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
    network.GenerateRandomSrcDst(4)

    # #Network medium
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

    #get network params
    n_list = network.GetNodeList()
    f_list = network.GetFlowList()
    sd_dict = network.GetSrcDstDict()
    a_list = network.GetArcList()
    c_dict = network.GetCapacityParam()

    nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.NonlinearCapacityConstraintsGenerator())
    rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, FLOW_BOUNDS)

    #initialize solvers
    rec_solver = sm.IpoptSolver()
    heur_solver = sm.DHeuristicSolver('Heap')

    tf.RunTest( network, rs_model, None, { 'Original': heur_solver, 'Recovered': rec_solver }, 
                solve_original=True, recover_feasible_original=True, validate_feasability=True, draw_solution=True)

#RunTest(111)

seed_data = [111, 222, 333, 444, 555]
#Run original:
for sd in seed_data:
    RunTest(sd)