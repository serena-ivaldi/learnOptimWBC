classdef (Abstract) AbstractController < handle
    
   properties(Abstract)
      subchains;
      references;
      torques;
      log;
   end
       
    
   methods(Abstract = true)
      Policy(obj,t,q,qd,Fc);
      GetTotalParamNum(obj);
   end
    
end