%% this class is an implementation of a vanilla version penalty function
%% 


% the idea is that we check the consraints inside the fitness function and
% than we update the information inside the object sometimes directly 


classdef  NoPenalty < Optimization.AbstractPenalty
    
   properties
       pop_size              % number of candidates for each generations
       n_constraint          % number of constraints to handle
       penalties             % matrix of penalties each column is a constraints each row is a candidate. here i have the actual violation or non violation of the constraints
       fitness_penalties     % this value is set to zero so in this way im not gonna change the value of the perfomance if i have a violation
       feasibility_vec
       feasibility            % in this vector i put a 1 is the vector satisfy all the constraints otherwise i put a zero
       constraints_functions % vector of functions handle for computing the constraints (actually a string vector that need str2func conversion to get the handles)
       constraints_type      % vector that specifies if the constraints is a equality or an inequality
       constraints_values    % vector that contains some constants that are used by the function in constraints_functions to compute the constraints_violation
       constraints_violation % amount of violations at the end of the rollout for the current candidate. it is a matrix where on the column we have the sampled violations and on the row we have the constraints
   end
       
   methods
       
       function obj = NoPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values)
           obj.pop_size = round(4 + 3 * log(search_space_dimension)); % this choice is the same used in CMAES when lambda is not spcified
           obj.n_constraint = length(constraints_functions);
           obj.constraints_functions = constraints_functions;
           obj.constraints_type = constraints_type;
           obj.constraints_values = constraints_values;
           obj.constraints_violation = zeros(obj.n_constraint,1);
           obj.penalties = zeros(1,obj.n_constraint);
           obj.feasibility_vec = zeros(1,obj.n_constraint);
           obj.feasibility = -1;
           obj.fitness_penalties = zeros(1,obj.pop_size);
       end
       
       % To call in Fitness
       function EvaluateConstraints(obj,input,iteration)
           % input is a column vector of value that we can use to compute the violation of the constraints 
           for i=1:length(input)
                obj.constraints_violation(i,iteration) = feval(obj.constraints_functions{1,i},input{1,i},obj.constraints_values(i));
           end     
       end
       % to call in ComputeConstraintsViolation and EvaluateCmaes 
       % here i set the value to zero so in this way the perfomanced is not
       % changed in EvaluateCMAES fir a -1 c_index
       function ComputePenalties(obj,c_index)  
          obj.fitness_penalties(c_index) = 0;
       end
       % To call in EvaluateCmaes
       % in this function i perform further elaborations with the
       % constraints violations. 
       % with c_index = -1 i manage the situation where i have to update just one candidate 
       % HERE PENALTIES REPRESENT THE REAL VALUE ASSOCIATED TO EACH
       % CONSTRAINTS
       % HERE c_index is not used and whatever value i put in the method is not gonna change the behaviour of the method 
       function ComputeConstraintsViolation(obj,c_index)
           % here i do not need to consider many candidates because this a
           % one sample per iteration method
           %obj.feasibility_vec=zeros(1,obj.n_constraint);
           for i=1:obj.n_constraint
               % i sum only the constraints violation it means that i have to discard value less then zero 
               % in this part i consider only the violation on each constraint for the overall experiments
               index = obj.constraints_violation(i,:)>0;
               %% TODO check isempty on the other constraints manager
               if(index ~= 0)
                    obj.penalties(1,i) = sum(obj.constraints_violation(i,index),2); 
                    obj.feasibility_vec(1,i) = -1;
               else
                   % otherwise i take all the index where T
                   index = obj.constraints_violation(i,:)<=0;
                   obj.penalties(1,i) = sum(obj.constraints_violation(i,index),2); 
                   obj.feasibility_vec(1,i) = 1;
               end
           end
           
           obj.feasibility = prod(obj.feasibility_vec);
           % i keep thiss part and i put penalties to zero to be compliant
           % with the way that i compute the fitness with the other method
           obj.ComputePenalties(1);   
       end  
        % to call in CMAES 
       % in this case im not using this method
       function [new_costs, new_performances] = FitnessWithPenalty(obj,policyId,old_costs,old_performances,cur_index)
          
       end
   end
end
