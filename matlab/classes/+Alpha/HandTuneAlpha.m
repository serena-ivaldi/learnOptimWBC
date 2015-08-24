classdef HandTuneAlpha < Alpha.AbstractAlpha
   
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
      
      function obj = HandTuneAlpha(alpha_func,time_struct)
         
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
      
       function Plot(obj)
         time = obj.time_struct.ti:obj.time_struct.step:obj.time_struct.tf;
            i=1;
            for t = time
                results(i) = obj.GetValue(t); 
                i=i+1;
            end
          plot(time,results)      
       end
      
      
      
   end
   
   methods (Static)
      
      %this function for t = ti is equal to 1 
      function val=TransFunc(ti,current_phase,transition_interval,t)
         val = 0.5 - 0.5*cos( ( ( t-ti(current_phase) ) / transition_interval(current_phase)) *pi );
      end
      
      %function that compute the value of the alpha function for all the time interval 
      function  all_value=ComputeValue(time,starting_value,ti,transition_interval)
         % i change ti in a way that after the last transition i will not
         % have any transition anymore
         ti = [ti inf];
         transition_flag = 0;
         current_phase = 1;
         all_value = [];
         % this value cqn be either 0 or  1si 
         steady_value = starting_value;
         for t=time
            %first of all i have to check if we are in transition or not
            if(t >= ti(current_phase) && ~transition_flag)
               transition_flag = 1;
            end
            % not in transition   
            if(~transition_flag)
                current_value = steady_value;
            else
               % compute the transition value
                transval=Alpha.HandTuneAlpha.TransFunc(ti,current_phase,transition_interval,t);
                % check if transval is bigger or equal than one 
                if(transval >=  1)
                   transval = 1;
                   transition_flag = 0;
                   current_phase = current_phase + 1;
                end
                
                 % in this block i assign the value of matrix_val in an app
                % matrix and than i update the value with the 
                if(steady_value == 0)
                   current_value = transval;
                else
                   current_value = 1 - transval;
                end
                % update steady_value
               if(transval >=  1)
                   if(steady_value == 0)
                       steady_value = 1;
                   else
                       steady_value = 0;
                   end
               end
              
            end
            
            all_value = [all_value ; current_value];
         end
         
         
      end   
      
      
      function  all_func = BuildChainAlpha(n_subchain,n_task,starting_value,ti,transition_interval,time_struct)
          % ONLY FOR SINGLE CHAINS!!
      
         
          % repcompute all the value of the alpha and i fit this data with
          % splines
          time = time_struct.ti:time_struct.step:time_struct.tf;
          for i = 1:n_subchain
              for j = 1:n_task
                  all_value = Alpha.HandTuneAlpha.ComputeValue(time,starting_value(i,j),ti(j,:,i),transition_interval(j,:,i));
                  all_func{i,j} = fit(time',all_value,'smoothingspline');
              end
          end   
      end
      
      
      function alphas = BuildCellArray(n_subchain,n_task,starting_value,ti,transition_interval,time_struct)
         
           all_func = Alpha.HandTuneAlpha.BuildChainAlpha(n_subchain,n_task,starting_value,ti,transition_interval,time_struct);

           % WORK ONLY FOR SINGLE CHAIN
           for i=1:n_subchain
              for j=1:size(all_func,2)
                 alphas{i,j} = Alpha.HandTuneAlpha(all_func{i,j},time_struct); 
              end
           end
      end
      
   end
   
   
   
end