from .coordgrad import CoordinatorGradient
from .gradstep import IStepRule
import pyomo.environ as pyo
import copy as cp

class CoordinatorSurrogateGradient(CoordinatorGradient):

    def __init__(self, step_rule = IStepRule()):
        super().__init__(step_rule)

    def GenerateSolvingPolicy(self):
        def SolvingPolicy(solve, cmodels_local):
            return solve(cmodels_local[ (self.n_iter - 1) % len(cmodels_local) ])
        self.solving_policy = SolvingPolicy
        return self.solving_policy
    
    def UpdateIterationData(self, cmodel):
        super().UpdateIterationData(cmodel)
        self.iteration_cmodel = cp.deepcopy(cmodel)

    def CheckExit(self):
        var_stop = sum([int(sc.CheckStop()) for sc in self.var_stop_crit]) == len(self.var_stop_crit)
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        if lm_stop or var_stop:
            self.SetBestSolution(self.iteration_cmodel)
            return True

class CoordinatorFsaGradient(CoordinatorSurrogateGradient):

    def __init__(self, step_rule = IStepRule()):
        super().__init__(step_rule)
        self.cmodel_partially_feasible = None
        self.cmodel_partially_feasible_value = float('-inf')
        self.restored_names = {}

    def GenerateSolvingPolicy(self):
        solving_policy_surrogate = super().GenerateSolvingPolicy()
        def SolvingPolicy(solve, cmodels_local):
            #for every relaxed constraint
            for relaxed_constraint_name in self.relaxed_constraints_violations:
                restored_constraint_names = self.restored_names[relaxed_constraint_name]
                violations_data = self.relaxed_constraints_violations[relaxed_constraint_name]
                for cml in cmodels_local:
                    #retrieve local model components
                    relaxed_constraint_local = getattr(cml,  relaxed_constraint_name)
                    relaxed_constraint_compare_operation_local = cml.Suffix[relaxed_constraint_local]['CompareOperation']
                    relaxed_constraint_expr_local_lhs = cml.Suffix[relaxed_constraint_local]['LHS']
                    #clear previously restored constraints
                    restored_constraint_list_local = getattr(cml, restored_constraint_names['RestoredConstraintListName'])
                    restored_constraint_list_local.clear()
                    for indx in violations_data:
                        violation = violations_data[indx]
                        #restore constraint if previously was feasible
                        if violation['Violated'] == False:
                            #previous lhs value of the decomposed unit must be substracted from feasible constraint value
                            value_lhs_local = pyo.value(relaxed_constraint_expr_local_lhs(cml, *indx))
                            #restored constraint
                            expr = relaxed_constraint_compare_operation_local(relaxed_constraint_expr_local_lhs(cml, *indx) + 
                                                                                violation['Value'] - value_lhs_local, 0)
                            if (expr is not False) and (expr is not True):
                                restored_constraint_list_local.add(expr)
            return solving_policy_surrogate(solve, cmodels_local)

        self.solving_policy = SolvingPolicy
        return self.solving_policy

    def UpgradeModel(self, amodels, relaxed_constraints_names):
        super().UpgradeModel(amodels, relaxed_constraints_names)
        for names in self.relaxation_names:
            restored_constraint_list_name = names[0] + 'RestoredList'
            self.restored_names[names[0]] = { 'RestoredConstraintListName': restored_constraint_list_name}

        for aml in amodels:
            for constr_name in self.restored_names:
                restored_name = self.restored_names[constr_name]['RestoredConstraintListName']
                resored_constraint_list = pyo.ConstraintList(name = restored_name)
                setattr(aml, restored_name, resored_constraint_list)

    def UpdateIterationData(self, cmodel):
        super().UpdateIterationData(cmodel)        
        self.relaxed_constraints_violations = {}
        self.violations_nmb = {}
        #for every relaxed constraint
        for names in self.relaxation_names:
            self.violations_nmb[names[0]] = 0
            self.relaxed_constraints_violations[names[0]] = {}
            #get constraints and expresssions
            relaxed_constraint = getattr(cmodel,  names[0])
            relaxed_set = getattr(cmodel, names[1])
            relaxed_constraint_expr_lhs = cmodel.Suffix[relaxed_constraint]['LHS']
            relaxed_constraint_expr_rhs = cmodel.Suffix[relaxed_constraint]['RHS']
            relaxed_constraint_compare_operation = cmodel.Suffix[relaxed_constraint]['CompareOperation']
            #check violation for every relaxed indx
            for indx in relaxed_set:
                value = pyo.value(relaxed_constraint_expr_lhs(cmodel, *indx) 
                                    - relaxed_constraint_expr_rhs(cmodel, *indx))
                acceptable_error = 1e-6
                if relaxed_constraint_compare_operation(value, 0) == True or relaxed_constraint_compare_operation(value - acceptable_error, 0) == True:
                    self.relaxed_constraints_violations[names[0]][indx] = { 'Violated': False, 'Value': value}
                else:
                    self.relaxed_constraints_violations[names[0]][indx] = { 'Violated': True, 'Value': 0}
                    self.violations_nmb[names[0]] += 1
        #save as if the solution is partially feasible
        if any([ nmb  == 0 for name, nmb in self.violations_nmb.items() ]):
            if self.cmodel_partially_feasible_value < self.obj_val:
                self.cmodel_partially_feasible = cp.deepcopy(cmodel)
                self.cmodel_partially_feasible_value = self.obj_val

    def CheckExit(self):
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        var_stop = sum([int(sc.CheckStop()) for sc in self.var_stop_crit]) == len(self.var_stop_crit)
        feasible = all([ nmb  == 0 for name, nmb in self.violations_nmb.items() ])
        if lm_stop or var_stop or feasible or (self.cmodel_partially_feasible is not None and self.n_iter >= 20):
            if self.cmodel_partially_feasible is not None:
                self.SetBestSolution(self.cmodel_partially_feasible)
            return True
