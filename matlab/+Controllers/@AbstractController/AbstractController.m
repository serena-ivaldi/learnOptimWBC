classdef (Abstract) AbstractController < handle
    
   properties(Abstract)
      subchains;
      references;
   end
       
    
   methods(Abstract = true)
      Policy(obj, t, q, qd);
   end
    
end