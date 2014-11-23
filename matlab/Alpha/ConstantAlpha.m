classdef ConstantAlpha < AbstractAlpha
   
   properties
      time_struct
      sample      % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
   end
   
   
   
   methods
      
      function obj = ConstantAlpha(value,time_struct)
          if(value>=0 && value<= 1)
            obj.sample = value;
          else
             error('alpha has to be between 0 and 1')  
          end
          
          obj.time_struct = time_struct;
          
      end
      
      %function that give the value of the alpha function given the current time 
      function val = GetValue(obj,t)
         val = obj.sample;
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
      function alphas = BuildCellArray(n_subchain,n_task,values,time_struct)
         
         for i=1:n_subchain
             for j =1:n_task
               alphas{i,j} = ConstantAlpha(values{i}(j),time_struct);
             end
         end
         
      end
   end
   
   
end