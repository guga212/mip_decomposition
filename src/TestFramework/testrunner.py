import ModelGenerator as mg
import ModelProcessors as mp
import NetworkFlow as net
import Drawer as dr
import copy as cp

def RunTest(network_graph, model_original, model_decomposer, 
            solvers, solve_original = False, solve_decomposed = False,
            validate_feasability = False, recover_feasible = False, 
            draw_progress = False, draw_solution = False):
    
    #overall results connection
    results = {}

    #solve decomposed problem
    if solve_decomposed:
        #solve decomposed problem
        solver_master = solvers['Master']
        solvers_local = solvers['Decomposed']
        solution = model_decomposer.Solve(solver_master, solvers_local)

        #show decomposition iterating data
        if draw_progress:
            data = model_decomposer.RecordedData
            dr.PlotIterationData('Objective', data['MasterObj'])
            for name, data in data['Multipliers'].items():
                dr.PlotIterationData(name, data)

        if solution is None:
            results['Decomposed'] = None
            print('###################!RESULTS!#############################')
            print('Decomposed:\nSolution was not found!')
            print('__________________________________________________________')
        else:
            objective, objective_dual, strains, routes, time = ( solution['Objective'], solution['ObjectiveDual'], 
                                                                solution['Strain'], solution['Route'], 
                                                                solution['Time'] )

            results['Decomposed'] = { 'Objective': objective, 'ObjectiveDual': objective_dual, 'Time': time }

            print('###################!RESULTS!#############################')
            print(f'Decomposed:\nObjective: {objective}, Objective Dual: {objective_dual}, Time: {time}')
            
            #validate constraints violations
            if validate_feasability:
                violations = mp.FindConstraintsViolation(model_original.cmodel, strains, routes)
                cc_vn = violations['capacity_constraints'][1]
                rc_vn = violations['route_constraints'][1]
                if cc_vn == 0 and rc_vn == 0:
                    print('Feasible')
                else:
                    print(f'Capacity constraint violations number: {cc_vn}')
                    print(f'Route constraint violations number: {rc_vn}')
            print('__________________________________________________________')

            #draw if needed
            if draw_solution:
                path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
                flow_list_sol = strains
                network_graph.SetPath(*path_list_sol)
                network_graph.SetFlows(*flow_list_sol)
                dr.PlotNetwork(network_graph, 'DECOMPOSED')

            #recover feasible for given routes
            if recover_feasible:
                solver = solvers['Recovered']
                solution = mp.RecoverFeasibleStrain(model_original, routes, solver)
                if solution is None:
                    results['Recovered'] = None
                    print('###################!RESULTS!#############################')
                    print('Recovered:\nSolution was not found!')
                    print('__________________________________________________________')
                else:
                    objective, strains, routes, time = ( solution['Objective'], solution['Strain'], 
                                                        solution['Route'], solution['Time'] )

                    results['Recovered'] = { 'Objective': objective, 'Time': time }

                    print('###################!RESULTS!#############################')
                    print(f'Recovered:\nObjective: {objective}, Time: {time}')
                    
                    #validate constraints violations
                    if validate_feasability:
                        violations = mp.FindConstraintsViolation(model_original.cmodel, strains, routes)
                        cc_vn = violations['capacity_constraints'][1]
                        rc_vn = violations['route_constraints'][1]
                        if cc_vn == 0 and rc_vn == 0:
                            print('Feasible')
                        else:
                            print(f'Capacity constraint violations number: {cc_vn}')
                            print(f'Route constraint violations number: {rc_vn}')
                    print('__________________________________________________________')

                    #draw if needed
                    if draw_solution:
                        path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
                        flow_list_sol = strains
                        network_graph.SetPath(*path_list_sol)
                        network_graph.SetFlows(*flow_list_sol)
                        dr.PlotNetwork(network_graph, 'RECOVERED')

    #solve original problem
    if solve_original:
        network_graph = cp.deepcopy(network_graph)
        solver = solvers['Original']
        solution = solver.Solve(model_original.cmodel)
        if solution is None:
            results['Original'] = None
            print('###################!RESULTS!#############################')
            print('Original:\nSolution was not found!')
            print('__________________________________________________________')
        else:
            objective, strains, routes, time = ( solution['Objective'], solution['Strain'], 
                                                solution['Route'], solution['Time'] )

            results['Original'] = { 'Objective': objective, 'Time': time }
                        
            print('###################!RESULTS!#############################')
            print(f'Original:\nObjective: {objective}, Time: {time}')

            #validate constraints violations
            if validate_feasability:
                violations = mp.FindConstraintsViolation(model_original.cmodel, strains, routes)
                cc_vn = violations['capacity_constraints'][1]
                rc_vn = violations['route_constraints'][1]
                if cc_vn == 0 and rc_vn == 0:
                    print('Feasible')
                else:
                    print(f'Capacity constraint violations number: {cc_vn}')
                    print(f'Route constraint violations number: {rc_vn}')
            print('__________________________________________________________')

            #draw if needed
            if draw_solution:
                path_list_sol =  [ [edge for edge, value in route.items() if value == 1] for route in routes ]
                flow_list_sol = strains
                network_graph.SetPath(*path_list_sol)
                network_graph.SetFlows(*flow_list_sol)
                dr.PlotNetwork(network_graph, 'ORIGINAL')

    #show figures if needed
    if draw_solution or draw_progress:
        dr.ShowAll()

    return results
