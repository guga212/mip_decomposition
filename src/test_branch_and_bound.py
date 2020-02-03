import BranchAndBound as bb
import NetworkFlow as net
import ModelGenerator as mg
import SolverManager as sm

def RuntTest(seed):
  
  #Init random generator
  net.NetworkGraph.SetRandomSeed(seed)

  #Define flow bounds
  FLOW_BOUNDS = (0.001, 3)
  CAPACITY_BOUNDS = (0.3, 1)

  #Network
  world_init_data = [ {'NodesNumber': 3, 'EdgesNumber': 8,  'ExternalEdgesNumber': 3 }, 
                        {'NodesNumber': 6, 'EdgesNumber': 4,  'ExternalEdgesNumber': 2 },
                        {'NodesNumber': 4, 'EdgesNumber': 6,  'ExternalEdgesNumber': 1 },
                        {'NodesNumber': 1, 'EdgesNumber': 3,  'ExternalEdgesNumber': 1 }
                      ]
  network = net.NetworkGraph.GenerateSmallWorld(world_init_data, *CAPACITY_BOUNDS)
  network.GenerateRandomSrcDst(4)


  bnb_solver = bb.BranchAndBoundSolverD(network, FLOW_BOUNDS)
  bnb_res = bnb_solver.Solve(log=True)
  print('BnB:')
  print(bnb_res)

  #Get network params
  n_list = network.GetNodeList()
  f_list = network.GetFlowList()
  sd_dict = network.GetSrcDstDict()
  a_list = network.GetArcList()
  c_dict = network.GetCapacityParam()
  nmg = mg.RsModelGenerator(mg.QuadObjectiveGenerator(), mg.RouteConstraintsGenerator(), mg.LinearCapacityConstraintsGenerator())
  rs_model = nmg.CreateCompletRsModel(f_list, sd_dict, n_list, a_list, c_dict, FLOW_BOUNDS)
  solver = sm.CplexSolver()
  solver_result = solver.Solve(rs_model.cmodel)
  print("SOLVER:")
  print(f"OBJECTIVE: {solver_result['Objective']}, TIME: {solver_result['Time']}")


seed_tab = [444]

for s in seed_tab:
  RuntTest(s)