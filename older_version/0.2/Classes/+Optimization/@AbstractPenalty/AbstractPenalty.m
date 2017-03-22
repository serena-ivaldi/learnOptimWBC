classdef (Abstract) AbstractPenalty < handle
    
    properties(Abstract)
       n_constraint          % number of constraints to handle
       fitness_penalties
       constraints_functions % vector of functions handle for computing the constraints (actually a string vector that need str2func conversion to get the handles)
       constraints_type      % vector that specifies if the constraints is a equality or an inequality. 1 is a disequalities 0 is an inequalities
       constraints_values    % vector that contains some constant that are used by the function in constraints_functions to compute the constraints_violation
       constraints_violation % amount of violation at the end of the rollout
    end
         
   methods(Abstract = true)
       % To call in the Fitness function
       EvaluateConstraints(obj,input,iteration)
       % To call in EvaluateCmaes
       % in this function i perform further elaborations witht the
       % constratins violations. In this case i compute the epsilon
       % normalized value of the sum of the constraints violation
       ComputeConstraintsViolation(obj,cur_candidates_index)
       % to call in CMAES                      
       FitnessWithPenalty(obj,policyId,old_costs,old_performances,cur_index)
   end  
   
   
   
end
