import ModelGenerator as mg
import collections as col
import random as rnd
import copy as cp


class BranchAndBoundSolver:

    class BranchData:
        def __init__(self, rs_model):
            self.rs_model = rs_model
            self.examined = False
            self.parent = None
            self.parent_indx = None
            self.lb = float('-inf')
            self.ub = float('inf')
            self.children_list = []

        @property
        def children_examined(self):
            children_len = len(self.children_list)
            children_examined_len = len([None for bcd in self.children_list if bcd.examined is True])
            ret_val = (children_len == children_examined_len)
            ret_val = ret_val and self.examined
            return ret_val

    def __init__(self, network, flow_bounds):
        self.log = True
        self.BranchingCutsName = 'BranchingCuts'
        self.required_precision = 1e-4
        # get network params
        n_list = network.GetNodeList()
        f_list = network.GetFlowList()
        sd_dict = network.GetSrcDstDict()
        a_list = network.GetArcList()
        c_dict = network.GetCapacityParam()
        # create model
        nmg = mg.RsModelGenerator(
                                    mg.QuadObjectiveGenerator(),
                                    mg.RouteConstraintsGenerator(),
                                    mg.NonlinearCapacityConstraintsGenerator()
                                  )
        self.rs_model = nmg.CreateCompletRsModel(
                                                    f_list, sd_dict, n_list,
                                                    a_list, c_dict, flow_bounds
                                                )

    def LogStatus(self, status):
        if self.log:
            print(status)

    def CreateBranchingModel(self):
        return cp.deepcopy(self.rs_model)

    def EstimateBounds(self, rs_model):
        return (None, None)

    def BranchModel(self, rs_model):
        return []

    def __GetStudiedBranches(self):
        
        def RowSearch():
            return len(self.branching_data[self.branch_level])
        def ColumnSearch():
            column_width = 2
            return min(len(self.branching_data[self.branch_level]), column_width)            

        if rnd.randint(1, 10) == 1:
            examined_branches_number = ColumnSearch()
        else:
            examined_branches_number = RowSearch()
        
        enumerated_branching = [ (indx, bd) for indx, bd in enumerate(self.branching_data[self.branch_level]) ]
        unexaminded_branches_data = [ (indx, bd) for indx, bd in enumerated_branching if bd.children_examined == False]
        if len(unexaminded_branches_data) == 0:
            return []
        rnd.shuffle(unexaminded_branches_data)
        ubd_slice = unexaminded_branches_data[0:examined_branches_number]
        return ubd_slice

    def __UpdateExaminedLevel(self):

        # level's children fully examined            
        for level, bd_list in self.branching_data.items():
            fully_explored_branches_data = [bd for bd in bd_list if bd.children_examined == True]            
            if len(fully_explored_branches_data) == len(bd_list):
                self.branching_levels_fully_explored[level] = True
            else:
                self.branching_levels_fully_explored[level] = False
        
        # update global lower bounds for every examined level
        for level, is_explored in self.branching_levels_fully_explored.items():
            if is_explored:
                level_lb_list = [bd.lb for bd in self.branching_data[level] if bd.lb is not None]
                if len(level_lb_list) > 0:
                    level_lb = min(level_lb_list)
                    self.prim_lb = max(self.prim_lb, level_lb)

        # no branches on the next level
        if len(self.branching_data[self.branch_level + 1]) == 0:
            nonexamined_levels = [lvl for lvl, ex in self.branching_levels_fully_explored.items() if ex == False]
            if len(nonexamined_levels) > 0:
                lowest_nonexamined_level = max(nonexamined_levels)
                self.branch_level = lowest_nonexamined_level
            else:
                self.out_of_branches = True
        else:
            self.branch_level += 1

    def __CheckExit(self):
        # check optimal
        if abs(self.prim_ub - self.prim_lb) <= self.required_precision:
            return True
        # check failed
        if self.out_of_branches is True:
            return False

    def Solve(self, log = True):

        # init options
        self.log = log

        # modify model
        self.rs_model_branching = self.CreateBranchingModel()

        # original problems date
        self.prim_ub = float('inf')
        self.prim_lb = float('-inf')
        self.total_time = 0

        # branching proccess data keeper
        self.branching_data = col.defaultdict(lambda: [])
        self.branching_levels_fully_explored = col.defaultdict(lambda: False)
        self.out_of_branches = False

        # zero level of branching
        self.branch_level = 0
        self.branching_data[self.branch_level] = [self.BranchData(self.rs_model_branching)]

        self.LogStatus(f'#######!!!!!!!$BnB$#######!!!!!!!')
        while True:

            self.LogStatus(f'_____________________________:')
            self.LogStatus(f'Level: {self.branch_level}. Global LB: {self.prim_lb}. Global UB: {self.prim_ub}')

            # estimate bounds for current level branches
            branching_data_current = self.__GetStudiedBranches()

            # iterate over every branch
            for bd_indx, bd in branching_data_current:

                # estimate bounds for current branch
                bd.lb, bd.ub = self.EstimateBounds(bd.rs_model)

                # update examination data
                bd.examined = True
                if bd.parent is None:
                    self.prim_lb = bd.lb
                else:
                    #update all examined parents lower bound
                    parent_iter = bd.parent
                    while parent_iter is not None:
                        if parent_iter.children_examined:
                            children_lb_list = [bcd.lb for bcd in parent_iter.children_list if bcd.lb is not None]
                            children_lb = min(children_lb_list) if len(children_lb_list) else float('inf')
                            if children_lb < parent_iter.lb: 
                                parent_iter.lb = children_lb
                                parent_iter = parent_iter.parent
                                continue
                        break

                        
                    

                self.LogStatus(f'###Branch[{bd_indx}]<-[{bd.parent_indx}]: lb = {bd.lb}, ub = {bd.ub}')

                # check exit feasibility
                if bd.lb is None or bd.ub is None:
                    continue

                # update global feasible bounds
                self.prim_ub = min(self.prim_ub, bd.ub)

                # check exit global feasible smaller then branch relaxed
                if bd.lb >= self.prim_ub:
                    continue

                # check exit optimality
                if bd.lb == bd.ub:
                    continue

                # add branching
                branching_models = self.BranchModel(bd.rs_model)                
                branching_children_data = [ self.BranchData(bm) for bm in branching_models ]
                for bcd in branching_children_data:
                    bcd.parent = bd
                    bcd.parent_indx = bd_indx
                    self.branching_data[self.branch_level+1].append(bcd)
                bd.children_list += branching_children_data
                    

            self.__UpdateExaminedLevel()

            exit_status = self.__CheckExit()
            if exit_status is not None:
                self.LogStatus(f'###BnB finished with: {exit_status}')
                break

        # exit algorithm
        self.LogStatus(f'###Results[LB* = {self.prim_lb}, UB* = {self.prim_ub}, Time = {self.total_time}')
        return {'LB': self.prim_lb, 'UB': self.prim_ub, 'Time': self.total_time }

