%% this object has to be use inside the fitness function defined in the same 
%% folder
%% this penalty function is an implementation of the method described in
%% Multidisciplinary Optimization in the Design of Future Space Launchers, Collange et al 2010.

% the idea is that we check the consraints inside the fitness function and
% than we update the information inside the object smetimes direcrtly 


classdef  Penalty < handle
    
   properties
       epsilon       % STRICTLY POSITIVE user defined constants
       weights
       r_i_feas      % vector of rate of feasible candidates for each generation
       r_i_average   % average value of the rate of feasible candidates  
       p_target
       pop_size
       gamma_matrix  % is a matrix of the epsilon normalized constrained values lambda x m where lambda is the number of candidates and m is the number of constraints   
   end
       
    
   methods
       
       function obj = Penalty(epsilon,pop)
           obj.epsilon = epsilon;
           obj.weights = 0.5*ones(1,lenght(epsilon));
           obj.pop_size = pop;
           
       end
       
     
       function penalty=ComputePenalty(obj,c_index)
           
           cur_row =obj.gamma_matrix(c_index,:);
           index = cur_row < 0;
           cur_row(1,index) = 0;
           penalty = cur_row*obj.weights;
           
       end
       
       function ComputeGamma(obj,c_index,constr_type,constr_vec)
           
           for i=1:lenght(constraints_vec)
               % inequality constraints
               if (constr_type==1)
                    obj.gamma_matrix(c_index,i) = (constr_vec(1,i) + obj.epsilon(i))/obj.epsilon(i);
               % equality constraints
               else
                   obj.gamma_matrix(c_index,i) = abs(constr_vec(1,i))/obj.epsilon(i);
               end
           end
           
       end
       
       
       
       function ComputeRfeas(obj)
           
           feasible_solution = 0;
           for i=1:obj.pop_size
               % im looking ofr all the candidates that are feasible
               index = obj.gamma_matrix(i,:)>1;
               if(lenght(index)==0)
                   feasible_solution = feasible_solution + 1; 
               end
           end
           obj.r_i_feas = [obj.r_i_feas feasible_solution/obj.pop_size];
           
       end
       
       function ComputeRfeasAverage(obj)
         obj.r_i_average =   mean(obj.r_i_feas,2);  
       end
       
       
       function ComputePtarget(obj,n)
           obj.p_target = (1/(obj.pop*n))^(1/())
       end
       
   end
    
end