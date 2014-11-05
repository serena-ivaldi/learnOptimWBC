classdef (Abstract) AbstractController < handle
    
   properties(Abstract)
      subchains;
      reference;
   end
       
    
   methods(Abstract = true)
      Policy(obj, t, q, qd);
   end
    
end