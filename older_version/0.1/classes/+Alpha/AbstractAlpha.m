classdef (Abstract) AbstractAlpha < handle
    
   properties(Abstract)
      time_struct            % struct with time_struct.ti time_struct.tf time_struct.step
      sample                 % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
   end
       
    
   methods(Abstract = true)
      %function that give the value of the alpha function given the current time 
      GetValue(obj,t);
      %function that compute the value of the alpha function given parameters
      ComputeNumValue(obj,theta);
      % function that give the number of parameters necessary for the alpha function
      GetParamNum(obj);
   end
    
   methods(Abstract, Static)
      % static method used for build a cell array of alpha function
      result = BuildCellArray
   end
   
   
end