import ModelGenerator as mg
import NetworkFlow as net
import SolverManager as sm
import ModelProcessors as mp
import Decomposers as dec
import Coordinators as cr
import TestFramework as tf
from enum import IntEnum

class NetworkSize(IntEnum):
    SMALL = 0
    LARGE = 1

class FormulationType(IntEnum):
    ORIGINAL = 0
    LINCONSTR = 1
    ALTERNATIVE = 2

class DecompositionType(IntEnum):
    EMPTY = 0
    DEMAND = 1
    SUBNET = 2
    SEPVAR = 3

class CoordinationType(IntEnum):
    EMPTY = 0
    PROXCP = 1
    SUBGRADLVL = 2


def RunTest(seed, network_size, formulation, decomposition = 0, coordination = 0):

    #desribe run
    network_description = ['Small', 'Large']
    formulations_description = ['Original', 'Linearized Constraints', 'Alternative']
    decomposition_description = ['Solve undecomposed', 'Demands', 'Subnetworks', 'Variable separating']
    coordination_description = ['Solve unrelaxed', 'Proximal cutting planes', 'Subgradient level evaluation']
    print('#######################################################')
    print(f'Seed is equal: {seed}')
    print(f'Network size: {network_description[network_size]}')
    print(f'Formulation: {formulations_description[formulation]}')
    print(f'Decomposition: {decomposition_description[decomposition]}')
    print(f'Coordination: {coordination_description[coordination]}')

    #define flow bounds
    FLOW_BOUNDS = (0.001, 3)
    CAPACITY_BOUNDS = (0.3, 1)

    #random network
    net.NetworkGraph.SetRandomSeed(seed)

    networks = [None, None]

    #Network small
    world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                        {'NodesNumber': 12, 'EdgesNumber': 32, 'ExternalEdgesNumber': 3 },
                        {'NodesNumber': 6, 'EdgesNumber': 22,  'ExternalEdgesNumber': 2 },
                         {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 2 }
                        ]
    networks[0] = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
    networks[0].GenerateRandomSrcDst(12)

    #Network medium
    world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                        {'NodesNumber': 12, 'EdgesNumber': 32, 'ExternalEdgesNumber': 3 },
                        {'NodesNumber': 6, 'EdgesNumber': 22,  'ExternalEdgesNumber': 2 },
                         {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 2 },
                         {'NodesNumber': 9, 'EdgesNumber': 16,  'ExternalEdgesNumber': 1 },
                         {'NodesNumber': 8, 'EdgesNumber': 20,  'ExternalEdgesNumber': 2 },
                         {'NodesNumber': 4, 'EdgesNumber': 12,  'ExternalEdgesNumber': 1 },
                         {'NodesNumber': 3, 'EdgesNumber': 6,  'ExternalEdgesNumber': 1 },
                        ]
    networks[1] = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
    networks[1].GenerateRandomSrcDst(32)
     
    network = networks[network_size]

    #get network params
    n_list = network.GetNodeList()
    f_list = network.GetFlowList()
    sd_dict = network.GetSrcDstDict()
    a_list = network.GetArcList()
    c_dict = network.GetCapacityParam()

    #select formulation
    if formulation == 0:
        nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.NonlinearCapacityConstraintsGenerator())    
    if formulation == 1:
        nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
    if formulation == 2:
        nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.ReformulatedConstraintsGenerator())
                
    rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, FLOW_BOUNDS)


    #initialize solvers
    opt_solver = sm.AslBaronSolver()
    mstr_solver = sm.IpoptSolver()
    rec_solver = sm.IpoptSolver()

    #select coordinator
    coordinator = None
    if coordination == 0:
        coordinator = None
    if coordination == 1:
        coordinator = cr.CoordinatorCuttingPlaneProximal(lm_min = -4.6, lm_max = 0.6)
    if coordination == 2:
        coordinator = cr.CoordinatorGradient(step_rule = cr.gradstep.ObjectiveLevelStepRule(3.2, 1.4))


    #select decomposer
    decomposer = None
    decomposed_solvers = []
    if decomposition == 0:
        decomposer = None
    if decomposition == 1:
        decomposed_solvers = [opt_solver for _ in f_list]
        decomposer = dec.FlowDecomposer(rs_model, coordinator)
    if decomposition == 2:
        decomposed_solvers = [opt_solver for _ in world_init_data]
        decomposer = dec.SubnetsDecomposer(rs_model, coordinator, network.GetWorldNodes())
    if decomposition == 3:
        decomposed_solvers = [opt_solver, opt_solver]
        decomposer = dec.SepVarDecomposer(rs_model, coordinator)

    #run computation
    tf.RunTest( network, rs_model, decomposer, { 'Original': opt_solver, 'Recovered': rec_solver, 'Master': mstr_solver, 
                                                'Decomposed': decomposed_solvers }, 
                solve_original=(coordination == 0), solve_decomposed=(coordination != 0), max_iter=55, 
                validate_feasability=True, recover_feasible=True, draw_progress=True, draw_solution=False
                )


RunTest(111, NetworkSize.SMALL, FormulationType.ALTERNATIVE, 
        DecompositionType.SEPVAR, CoordinationType.PROXCP)