import pyomo.environ as pyo
import copy as cp

def RelaxConstraints(amodel, relaxed_constraints_names):
    """ 
        Function for relaxation specific constraint.
        Accepts abstract model, and dictionary
        of the relaxed constraints names and sets.
    """

    for names in relaxed_constraints_names:
        relaxed_constraint_name = names[0]
        relaxed_constraint_set_name = names[1]
        relaxed_constraint = getattr(amodel, relaxed_constraint_name)
        relaxed_constraint_rule_prev = relaxed_constraint.rule
        def RelaxedCapacityConstraintRule(model, *args):
            concrete_relaxed_constraint_set = getattr(model, relaxed_constraint_set_name)
            if concrete_relaxed_constraint_set.is_constructed() == False:
                concrete_relaxed_constraint_set.construct()
            if args in concrete_relaxed_constraint_set:
                return pyo.Constraint.Feasible
            return relaxed_constraint_rule_prev(model, *args)
        relaxed_constraint.rule = RelaxedCapacityConstraintRule