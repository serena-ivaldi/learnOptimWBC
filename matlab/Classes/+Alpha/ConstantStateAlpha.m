% in this class i define a column of value for each task. each element of
% the column represent a 

classdef ConstantStateAlpha < Alpha.AbstractAlpha
   
   properties
      time_struct                % time struct is not used but i could be used in the future 
      sample                     % array of values (one for each state) associated to one task ( row = task column = states)
      range                      % necessary because i call it inside instance cmaes (it seems to be not used anymore)
      param    
   end
   
   
   
   methods
      % here value represents the set of coefficients that the task assumes
      % while is in different state during the experiment
      function obj = ConstantStateAlpha(value,value_range,time_struct)
         
          obj.sample = value;
          obj.time_struct = time_struct;
          % only to be compliant with the whole structure
          obj.range = value_range;
          
      end
      
      % function that give the value of the alpha function given the current time 
      function val = GetValue(obj,state)
         val = obj.sample(state);
      end   
      % function that update the value to test
      function ComputeNumValue(obj,theta)
          obj.sample = theta;
      end
            
      % function that give the number of parameters necessary for the alpha function
      function r = GetParamNum(obj)
         r = length(obj.sample(1,:));
      end   
   end
   
   methods (Static)
      % here the constructor accept a matrix of array as values ( row = task column = states)
      % here we remove subchain because this class is built around the
      % simulink simulator with the icub humanoids
      function alphas = BuildCellArray(n_task,values,value_range,time_struct)
         
         %for i=1:n_subchain
             ii = 1;
             for j =1:n_task
               alphas{ii,j} = Alpha.ConstantStateAlpha(values(j,:),value_range,time_struct);
             end
         %end
         
      end
   end
   
   
end