from .coordgrad import CoordinatorGradient
from .gradstep import IStepRule
import pyomo.environ as pyo
import copy as cp

class CoordinatorSurrogateGradient(CoordinatorGradient):

    def __init__(self, step_rule = IStepRule()):
        super().__init__(step_rule)

    def GenerateSolvingPolicy(self):
        def SolvingPolicy(solve, cmodels_local):
            if solve(cmodels_local[self.n_iter % len(cmodels_local)]) == False:
                return False
        self.solving_policy = SolvingPolicy
        return self.solving_policy

class CoordinatorFsaGradient(CoordinatorSurrogateGradient):

    def __init__(self, step_rule = IStepRule()):
        super().__init__(step_rule)
        self.restored_names = {}

    def GenerateSolvingPolicy(self):
        solving_policy_surrogate = super().GenerateSolvingPolicy()
        def SolvingPolicy(solve, cmodels_local):
            #for every relaxed constrain
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
                            restored_constraint_list_local.add(expr)
            return solving_policy_surrogate(solve, cmodels_local)

        self.solving_policy = SolvingPolicy
        return self.solving_policy

    def UpgradeModel(self, amodel_local, relaxed_constraints_names):
        super().UpgradeModel(amodel_local, relaxed_constraints_names)
        for names in self.relaxation_names:
            restored_constraint_list_name = names[0] + 'RestoredList'
            self.restored_names[names[0]] = { 'RestoredConstraintListName': restored_constraint_list_name}
            resored_constraint_list = pyo.ConstraintList(name = restored_constraint_list_name)
            setattr(amodel_local, restored_constraint_list_name, resored_constraint_list)

    def UpdateIterationData(self, cmodel):
        super().UpdateIterationData(cmodel)

        self.relaxed_constraints_violations = {}
        self.violations_nmb = 0
        #for every relaxed constraint
        for names in self.relaxation_names:
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
                if relaxed_constraint_compare_operation(value, 0) == True:
                    self.relaxed_constraints_violations[names[0]][indx] = { 'Violated': False, 'Value': value}
                else:
                    self.relaxed_constraints_violations[names[0]][indx] = { 'Violated': True, 'Value': 0}
                    self.violations_nmb += 1
        #save as best if the solution is feasible
        if self.violations_nmb == 0:
            self.SetBestSolution(cmodel)

    def CheckExit(self):
        obj_stop = self.obj_stop_crit.CheckStop()
        lm_stop = sum([int(sc.CheckStop()) for sc in self.lagr_mult_stop]) == len(self.lagr_mult_stop)
        feasible = (self.violations_nmb == 0)
        if obj_stop or lm_stop or feasible:
            return True
