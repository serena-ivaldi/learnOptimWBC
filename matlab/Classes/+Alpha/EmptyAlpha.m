classdef EmptyAlpha < Alpha.AbstractAlpha
   
   properties
      time_struct
      sample      % value for a specific set of theta and sampling time 
      range       % necessary because i call it inside instance cmaes
      param    
   end
   
   
   
   methods
      
      function obj = EmptyAlpha(value,value_range,time_struct)
           obj.param        = [];
           obj.range        = [];
           obj.time_struct  = [];
      end
      
      %function that give the value of the alpha function given the current time 
      function val = GetValue(obj,t)
         val = [];
      end   
      %function that compute the value of the alpha function given parameters
      function ComputeNumValue(obj,theta)
         
      end
      % function that give the number of parameters necessary for the alpha function
      function r = GetParamNum(obj)
         r = 0;
      end   
   end
   
   methods (Static)
      function alphas = BuildCellArray(n_subchain,n_task,values,value_range,time_struct)
      alphas = [];
      end
   end
   
   
end