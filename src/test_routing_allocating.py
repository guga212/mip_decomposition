import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import ModelProcessors as mp
import Decomposers as dec
import Coordinators as cr
import time as t


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
                        {'NodesNumber': 12, 'EdgesNumber': 32, 'ExternalEdgesNumber': 3 },
                        {'NodesNumber': 6, 'EdgesNumber': 22,  'ExternalEdgesNumber': 2 },
                         {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 2 }
                        ]
    network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
    network.GenerateRandomSrcDst(12)

    # #Network medium
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


    start_time_lb = t.perf_counter()
    estimate_result_lb = mp.EstimateObjectvie(rs_model, False)
    elapsed_time_lb = t.perf_counter() - start_time_lb
    objective_lb, strains_lb, routes_lb, time_lb = (estimate_result_lb['Objective'], estimate_result_lb['Strain'], 
                                                        estimate_result_lb['Route'], estimate_result_lb['Time'] )
    print('###################!RESULTS!#############################')
    print(f'LB:\nObjective: {objective_lb}, Time: {time_lb}, Total time: {elapsed_time_lb}')
    violations = mp.FindConstraintsViolation(rs_model.cmodel, strains_lb, routes_lb)
    cc_vn = violations['capacity_constraints'][1]
    rc_vn = violations['route_constraints'][1]
    if cc_vn == 0 and rc_vn == 0:
        print('Feasible')
    else:
        print(f'Capacity constraint violations number: {cc_vn}')
        print(f'Route constraint violations number: {rc_vn}')
    print('__________________________________________________________')


    start_time_ub = t.perf_counter()
    estimate_result_ub = mp.EstimateObjectvie(rs_model, True)
    elapsed_time_ub = t.perf_counter() - start_time_ub
    objective_ub, strains_ub, routes_ub, time_ub = (estimate_result_ub['Objective'], estimate_result_ub['Strain'], 
                                                        estimate_result_ub['Route'], estimate_result_ub['Time'] )
    print('###################!RESULTS!#############################')
    print(f'UB:\nObjective: {objective_ub}, Time: {time_ub}, Total time: {elapsed_time_ub}')
    violations = mp.FindConstraintsViolation(rs_model.cmodel, strains_ub, routes_ub)
    cc_vn = violations['capacity_constraints'][1]
    rc_vn = violations['route_constraints'][1]
    if cc_vn == 0 and rc_vn == 0:
        print('Feasible')
    else:
        print(f'Capacity constraint violations number: {cc_vn}')
        print(f'Route constraint violations number: {rc_vn}')
    print('__________________________________________________________')

#RunTest(111)

seed_data = [111, 222, 333, 444, 555]
#Run original:
for sd in seed_data:
    RunTest(sd)