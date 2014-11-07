classdef (Abstract) AbstractController < handle
    
   properties(Abstract)
      subchains;
      references;
      torques;
   end
       
    
   methods(Abstract = true)
      Policy(obj, t, q, qd);
   end
    
end