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
       fitness_penalties     % is a matrix of the epsilon normalized constrained values (lambda x m) where lambda is the number of candidates and m is the number of constraints   
       n_constraint          % number of constraints to handle
       constraints_functions % vector of functions handle for computing the constraints (actually a string vector that need str2func conversion to get the handles)
       constraints_type      % vector that specifies if the constraints is a equality or an inequality. 1 is a disequality, 0 is an equality
       constraints_values    % vector that contains some constant that are used by the function in constraints_functions to compute the constraints_violation
       constraints_violation % amount of violation at the end of the rollout
       
   end
       
    
   methods
       
       function obj = AdaptivePenalty(epsilon,n_generation,search_space_dim,constraints_functions,constraints_type,constraints_values)
           obj.epsilon = epsilon;
           obj.weights = ones(n_generation,length(epsilon))*100;
           obj.pop_size =  round(4 + 3 * log(search_space_dim));  % this choice is the same used in CMAES when lambda is not spcified
           obj.search_space_dim = search_space_dim;
           obj.n_constraint = length(epsilon); 
           obj.r_i_average = zeros(1,obj.n_constraint);
           obj.r_i_feas = zeros(n_generation,obj.n_constraint);
           obj.p_target = 0.5; % is fixed but is not entirely correct  
           obj.constraints_functions = constraints_functions;
           obj.constraints_type = constraints_type;   
           obj.constraints_values = constraints_values;   
           obj.constraints_violation = zeros(obj.n_constraint,1);
           obj.fitness_penalties = zeros(obj.pop_size,obj.n_constraint);
       end
       % To call in Fitness
       function EvaluateConstraints(obj,input,iteration)
           % input is a column vector of value that we can use to compute the violation of the constraints 
           for i=1:length(input)
                obj.constraints_violation(i,iteration) = feval(obj.constraints_functions{1,i},input{1,i},obj.constraints_values(i));
           end     
       end
       % to call in ComputeConstraintsViolation and EvaluateCmaes or FitnessWithPenalty
       % through switch_flag i control if im computing penalties in
       % FitnessWithPenalty or in ComputeConstraintsViolation
       function penalties=ComputePenalties(obj,cur_index,switch_flag)
           penalties = zeros(1,obj.pop_size);
           if(switch_flag)
              for i=1:obj.pop_size
                  cur_row =obj.fitness_penalties(i,:);
                  % i zeroing all the value minus than zero
                  index = cur_row < 0;
                  cur_row(1,index) = 0;
                  penalties(1,i) = (cur_row.^(2))*obj.weights(cur_index,:)';  
              end
           else
              cur_row =obj.fitness_penalties(1,:);
              % i zeroing all the value minus than zero
              index = cur_row < 0;
              cur_row(1,index) = 0;
              % in the obj.fitness_penalties(1) i store the current fitness
              % penalty for the mean candidates and it is a solution that i
              % introduced to be compliants with the way that we use to
              % maanage the mean solutions
              obj.fitness_penalties(1) = (cur_row.^(2))*obj.weights(cur_index,:)';  
           end
       end
       % To call in EvaluateCmaes
       % in this function i perform further elaborations witht the
       % constraints violations. In this case i compute the epsilon
       % normalized value of the sum of the constraints violation
       function ComputeConstraintsViolation(obj,c_index)
           if(c_index>0)    
               for i=1:obj.n_constraint 
                   % i have to sum only the positive value that will rapresents
                   % the positive value. if i have no positive value i have
                   % to sum up all the negative
                   index = obj.constraints_violation(i,:)>0;
                   if(any(index))
                     overall_const_viol = sum(obj.constraints_violation(i,index),2);
                   else
                     overall_const_viol = sum(obj.constraints_violation(i,:),2);
                   end
                   % inequality constraints
                   if (obj.constraints_type(i)==1)    
                        obj.fitness_penalties(c_index,i) = (overall_const_viol + obj.epsilon(i))/obj.epsilon(i);
                   % equality constraints
                   else
                        obj.fitness_penalties(c_index,i) = overall_const_viol/obj.epsilon(i);
                   end
               end
           else
               for i=1:obj.n_constraint
                   % i have to sum only the positive value that will rapresents
                   % the positive value. if i have no positive value i have
                   % to sum up all the negative
                   index = obj.constraints_violation(i,:)>0;
                   if(any(index))
                     overall_const_viol = sum(obj.constraints_violation(i,index),2);
                   else
                     overall_const_viol = sum(obj.constraints_violation(i,:),2);
                   end
                   % inequality constraints
                   if (obj.constraints_type(i)==1)    
                        obj.fitness_penalties(1,i) = (overall_const_viol + obj.epsilon(i))/obj.epsilon(i);
                   % equality constraints
                   else
                        obj.fitness_penalties(1,i) = overall_const_viol/obj.epsilon(i);
                   end
               end
               % i have to change the sign because i need cur index to
               % access the right entry in  the weights matrix
               % and i need it negative to distinguish between the
               % execution to compute the mean fitness of the mean
               % candidates (cindex here is -1)
               c_index = - c_index;
               obj.ComputePenalties(c_index,false);
           end
       end
       
       % to call in CMAES                      
       function [new_costs, new_performances] = FitnessWithPenalty(obj,policyId,old_costs,old_performances,cur_index)
           obj.UpHandleConstraints(cur_index)
           penalties = obj.ComputePenalties(cur_index,true);
           % for the performance because im considering negative value i have to subtract the penalties  
           new_performances = old_performances - penalties';
           % for the cost i have to add the penalties in order to obtain
           % the corrected cost
           new_costs = old_costs;
           new_costs(1,policyId - obj.pop_size : policyId - 1) = old_costs(1, policyId - obj.pop_size : policyId - 1) + penalties;  
       end
       
   end
   
   % private methods
   methods (Access = private)
       function UpHandleConstraints(obj,cur_index)
           obj.ComputeRfeas(cur_index);
           obj.ComputeRfeasAverage(cur_index);
           obj.ComputePtarget() 
           obj.UpdateWeights(cur_index);
       end
       
       function ComputeRfeas(obj,cur_index)
           for i=1:obj.n_constraint
               % Im looking for all the candidates that are feasible in the
               % constraint i
               index = obj.fitness_penalties(:,i) <= 1;
               obj.r_i_feas(cur_index,i) = sum(index)/obj.pop_size; 
           end          
       end
       
       function ComputeRfeasAverage(obj,cur_index)
           if(cur_index <= obj.search_space_dim+2)    
               for i=1:obj.n_constraint
                  obj.r_i_average(1,i) = mean(obj.r_i_feas(1:cur_index,i),1);  
               end
           else
               for i=1:obj.n_constraint
                  obj.r_i_average(1,i) = mean(obj.r_i_feas( cur_index - (obj.search_space_dim + 1):cur_index , i ),1);  
               end  
           end
       end
       
       function ComputePtarget(obj)
           index_average_active_constraints = obj.r_i_average < 1;
           obj.p_target = (1/(obj.pop_size*obj.search_space_dim))^(1/( sum(index_average_active_constraints) + 10^(-6) ));
       end
       
       function UpdateWeights(obj,cur_index)
          for i=1:obj.n_constraint
            obj.weights(cur_index,i) =  obj.weights(cur_index-1,i) * ( exp(obj.p_target - obj.r_i_feas(cur_index-1,i)) )^(1/obj.search_space_dim);  
          end
       end
       
   end  
    
end
