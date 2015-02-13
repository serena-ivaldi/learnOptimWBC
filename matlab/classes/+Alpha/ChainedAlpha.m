classdef ChainedAlpha < Alpha.AbstractAlpha
   
   properties
      time_struct
      sample              % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
      range               % necessary because i call it inside instance cmaes
      %matrix_value        % this field is rapresented by a matrix T(n x n x k) where n is the number of tasks and k is the number change in task priority
      %transition_matrix   % matrix obtained by subtracting matrix inside matrix value  
      %ti                  % starting point for each transition number of transition = (k-1) row vector
      %transition_interval % lenght in time of the transition interval
      %current_phase       % is an index that say which transition time we have to check to start a new transition 
      %transition_flag     % when is 1 im in transition so i have to use the transition function i exit from transition when the value that we obtain are close to 0 or 1;
      %all_value           % array of value used to train spline and for debug
      alpha_func          % is the vector of all the value that we have for all time
      
   end
   
   
   
   methods
      
      function obj = ChainedAlpha(alpha_func,time_struct)
         
          obj.time_struct = time_struct;
          % i add inf to vector of time to avoid that after the last
          % transition i came back inside the transition block
          %obj.ti = [ti,inf];
          %obj.transition_interval = transition_interval;
           % only to be compliant with the whole structure
          obj.range = [0 1];
          obj.alpha_func = alpha_func;
           
      end
      
      
      
      
      function result = GetValue(obj,t)
         result = feval(obj.alpha_func,t);
      end
      
      
      
      
      %function that compute the value of the alpha function given parameters
      function ComputeNumValue(obj,theta)
          
      end
      % function that give the number of parameters necessary for the alpha function
      % here i dont have parameters
      function r = GetParamNum(obj)
         r = 0;
      end   
   end
   
   methods (Static)
      
      %this function for t = ti is equal to 1 
      function val=TransFunc(ti,current_phase,transition_interval,t)
         val = 0.5 - 0.5*cos( ( ( t-ti(current_phase) ) / transition_interval ) *pi );
      end
      
      %function that give the value of the alpha function with the current time 
      function  all_value=ComputeValue(time,ti,transition_matrix,matrix_value,transition_interval)
         
         transition_flag = 0;
         current_phase = 1;
         all_value = [];
         for t=time
            %first of all i have to check if we are in transition or not
            if(t >= ti(current_phase) && ~transition_flag)
               transition_flag = 1;
            end

            % not in transition
            if(~transition_flag)
               % transform the matrix_val in a vector by stacking the row
               % transposed   
               current_value = reshape(matrix_value',1,[]); 

            % in transition   
            else
               % compute the transition value
                transval=Alpha.ChainedAlpha.TransFunc(ti,current_phase,transition_interval,t);

                % -1 is the placeholder for the alpha that is increasing
                ind = find(transition_matrix(:,:,current_phase) == -1);
                [row,col] = ind2sub(size(transition_matrix(:,:,current_phase)), ind);

                % check if transval is bigger or equal than one 
                if(transval >=  1)
                   transval = 1;
                   transition_flag = 0;
                   current_phase = current_phase + 1;
                   % update matrix_val
                   matrix_value =  matrix_value - transition_matrix(:,:,current_phase - 1);
                end

                % in this block i assign the value of matrix_val in an app
                % matrix and than i update the value with the 
                app_matrix_val = matrix_value;
                app_matrix_val(row,col) = transval;
                app_matrix_val(col,row) = 1 - transval;
                % transform the matrix_val in a vector by stacking the row
                % transposed
                current_value = reshape(app_matrix_val',1,[]); 


            end
            all_value = [all_value ; current_value];
         end
         
         
      end   
      
      
      function  all_func = BuildChainAlpha(matrix_value,ti,transition_interval,time_struct)
          % ONLY FOR SINGLE CHAINS!!
          matrix_value_start= matrix_value(:,:,1);
          
          % i add inf to vector of time to avoid that after the last
          % transition i came back inside the transition block
          ti = [ti,inf];
          
          % compute transition matrix 
          for i = 2 : size(matrix_value,3)
             transition_matrix(:,:,i-1) = matrix_value(:,:,i-1) - matrix_value(:,:,i);
          end
        
         
          % repcompute all the value of the alpha and i fit this data with
          % splines
          time = time_struct.ti:time_struct.step:time_struct.tf;
          
          all_value=Alpha.ChainedAlpha.ComputeValue(time,ti,transition_matrix,matrix_value_start,transition_interval);
          
          
          for i = 1:size(all_value,2)
              all_func{i} = fit(time',all_value(:,i),'smoothingspline');
          end   
      end
      
      
      function alphas = BuildCellArray(n_subchain,matrix_value,ti,transition_interval,time_struct)
         
               all_func = Alpha.ChainedAlpha.BuildChainAlpha(matrix_value,ti,transition_interval,time_struct);
         
               % WORK ONLY FOR SINGLE CHAIN
               for i=1:n_subchain
                  for j=1:size(all_func,2)
                     alphas{i,j} = Alpha.ChainedAlpha(all_func{j},time_struct); 
                  end
               end
      end
      
   end
   
   
end