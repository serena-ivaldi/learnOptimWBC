classdef (Abstract) AbstractController < handle
    
   properties(Abstract)
      subchains;
      reference;
      policy;
   end
       
    
   methods(Abstract = true)
      Policy(obj, t, q, qd);
   end
    
end