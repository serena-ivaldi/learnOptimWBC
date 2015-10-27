classdef (Abstract) AbstractPenalty < handle
    
%    properties(Abstract)
%       n_constraint          % number of constraints to handle
%    end
         
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