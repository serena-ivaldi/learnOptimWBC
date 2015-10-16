classdef (Abstract) AbstractPenalty < handle
    
%    properties(Abstract)
%       time_struct            % struct with time_struct.ti time_struct.tf time_struct.step
%       sample                 % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
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