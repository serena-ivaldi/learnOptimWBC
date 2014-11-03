classdef  References < handle
    
   properties
      subchain;
      type;
      trajectories;
   end
       
    
   methods
      
      function obj = References(subchain,type)
         if (isa(subchain,'function_handle'))
            obj.subchain = subchain;
         else
            error('it is necessary to use a handle of the object bot!')   
         end
         obj.type = type;
      end  
      
   end
    
end