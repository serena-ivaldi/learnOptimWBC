classdef ConstantStateAlpha < Alpha.AbstractAlpha
   
   properties
      time_struct                % time struct is not used but i could be used in the future 
      constant_value_in_state    % array of values (one for each state) associated to one task
      range                      % necessary because i call it inside instance cmaes (it seems to be not used anymore)
      param    
   end
   
   
   
   methods
      
      function obj = ConstantStateAlpha(value,value_range,time_struct)
         
          obj.sample = value;
          obj.time_struct = time_struct;
          % only to be compiant with the whole structure
          obj.range = value_range;
          
      end
      
      %function that give the value of the alpha function given the current time 
      function val = GetValue(obj,state)
         val = obj.sample(state);
      end   
      %function that compute the value of the alpha function given parameters
      function ComputeNumValue(obj,theta)
          
      end
      % function that give the number of parameters necessary for the alpha function
      function r = GetParamNum(obj)
         r = length(obj.constant_value_in_state);
      end   
   end
   
   methods (Static)
      % here the cosntructor accept a matrix of array as values ( row = task column = states)
      % here we remove subchain because this class is built around the
      % simulink simulator with the icub humanoids
      function alphas = BuildCellArray(n_task,values,value_range,time_struct)
         
         %for i=1:n_subchain
             ii = 1;
             for j =1:n_task
               alphas{ii,j} = Alpha.ConstantAlpha(values(j,:),value_range,time_struct);
             end
         %end
         
      end
   end
   
   
end