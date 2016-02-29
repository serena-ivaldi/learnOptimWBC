classdef (Abstract) AbstractController < handle
    
   properties(Abstract)
      subchains;
      references;
      torques;
   end
       
    
   methods(Abstract = true)
      Policy(obj,t,q,qd,Fc);
      GetTotalParamNum(obj);
   end
    
end