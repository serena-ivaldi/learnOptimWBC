
%% this penalty function is an implementation of the method described in
%% Multidisciplinary Optimization in the Design of Future Space Launchers, Collange et al 2010.

% the idea is that we check the consraints inside the fitness function and
% than we update the information inside the object smetimes direcrtly 


classdef  AdaptivePenalty < Optimization.AbstractPenalty
    
   properties
       epsilon               % STRICTLY POSITIVE user defined constants
       weights       
       r_i_feas              % vector of rate of feasible candidates for each generation
       r_i_average           % average value of the rate of feasible candidates  
       p_target
       pop_size
       search_space_dim      % dimesion of the search space of our problem
       gamma_matrix          % is a matrix of the epsilon normalized constrained values lambda x m where lambda is the number of candidates and m is the number of constraints   
       n_constraint
       constraints_functions % vector of functions handle for computing the constraints 
       constraints_type      % vector that specifies if the constraints is a equality or an inequality. 1 is a disequalities 0 is an inequalities
       constraints_values    % vector that contains some constant that are used by the function in constraints_functions to compute the constraints_violation
       constraints_violation % amount of violation at the end of the rollout
       
   end
       
    
   methods
       
       function obj = AdaptivePenalty(epsilon,n_generation,search_space_dim)
           obj.epsilon = epsilon;
           obj.weights = ones(1,length(epsilon))*(1/length(epsilon));
           obj.pop_size =  round(4 + 3 * log(search_space_dimension));  % this choice is the same used in CMAES when lambda is not spcified
           obj.search_space_dim = search_space_dim;
           obj.n_constraint = length(epsilon); 
           obj.r_i_average = zeros(1,obj.n_constraint);
           obj.r_i_feas = zeros(n_generation,obj.n_constraint);
           obj.p_target = 0.5; % is fixed but is not entirely correct   
       end
       
       % To call in Fitness
       function EvaluateConstraints(obj,input,iteration)
           % input is a column vector of value that we can use to compute the violation of the constraints 
           for i=1:length(input)
                obj.constraints_violation(i,iteration) = feval(obj.constraints_functions{1,i},input(i,1),obj.constraints_values(i));
           end     
       end
       
       % To call in EvaluateCmaes
       % in this function i perform further elaborations witht the
       % constratins violations. In this case i compute the epsilon
       % normalized value of the sum of the constraints violation
       function ComputeConstraintsViolation(obj,cur_candidates_index)
           for i=1:length(constraints_vec)
               % i have to sum only the positive value that will rapresents
               % the positive value
               overall_const_viol =  sum(obj.constraints_violation(i,:),2); 
               % inequality constraints
               if (obj.constraints_type(i)==1)    
                    obj.gamma_matrix(cur_candidates_index,i) = (overall_const_viol + obj.epsilon(i))/obj.epsilon(i);
               % equality constraints
               else
                   obj.gamma_matrix(cur_candidates_index,i) = overall_const_viol/obj.epsilon(i);
               end
           end
       end
       
       % to call in CMAES                      
       function [new_costs, new_performances] = FitnessWithPenalty(obj,policyId,old_costs,old_performances,cur_index)
           
           obj.UpHandleConstraints(cur_index)
           penalties = zeros(obj.pop_size,1);
           for i=1:obj.pop_size
               cur_row =obj.gamma_matrix(i,:);
               index = cur_row < 0;
               cur_row(1,index) = 0;
               penalties(i) = (cur_row^(2))*obj.weights;  
           end
           % for the performance because im considering negative value i have to subtract the penalties  
           new_performances = old_performances - penalties;
           % for the cost i have to add the penalties in order to obtain
           % the corrected cost
           new_costs = old_costs;
           new_costs(1,policyId - obj.pop_size : policyId - 1) = old_costs(1, policyId - obj.pop_size : policyId - 1) + penalties;  
       end
       
   end
   
   % private methods
   methods (Access = private)
       % to call in CMAES
       function UpHandleConstraints(obj,cur_index)
           obj.ComputeRfeas(cur_index);
           obj.ComputeRfeasAverage();
           %obj.ComputePtarget() for now ptarget is fixed
           obj.UpdateWeights(cur_index);
       end
       
        function ComputeRfeas(obj,cur_index)
           for i=1:obj.n_constraint
               % Im looking for all the candidates that are feasible
               index = obj.gamma_matrix(i,:) <= 1;
               obj.r_i_feas(cur_index,i) = length(index)/obj.pop_size; 
           end          
       end
       
       function ComputeRfeasAverage(obj)
           if(length(obj.r_i_feas)<obj.search_space_dim+2)    
               for i=1:obj.n_constraint
                  obj.r_i_average(1,i) = mean(obj.r_i_feas(:,1),1);  
               end
           else
               for i=1:obj.n_constraint
                  obj.r_i_average(1,i) = mean(obj.r_i_feas(end - obj.search_space_dim -1 , 1 ),1);  
               end  
           end
       end
       
       function ComputePtarget(obj)
           obj.p_target = (1/(obj.pop*obj.search_space_dim))^(1/(k));
       end
       
       function UpdateWeights(obj,cur_index)
          for i=1:obj.n_constraint
            obj.weights(1,i) =  obj.weights(1,i) * exp(obj.p_target - obj.r_i_feas(cur_index,i));  
          end
       end
       
    end 
    
end