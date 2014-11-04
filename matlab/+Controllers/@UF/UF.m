classdef  UF < AbstractController
    
   properties
      references;       % object that contains the reference trajectory for each tasks; 
      subchains;        % object that contains the subchain of the robot and the J dot for each subchain;   
      %alpha;           % vector of weight function
      N;                % vector of metric 
      K_p               % vector of matrix of proportional gain
      K_d               % vector of matrix of derivative gain
      combine_rule      % projector or sum 
      policy = @Policy; % pointer to the function policy
   end


   methods
      function obj = UF(sub_chains,references,N)
         obj.subchain = sub_chains;
         obj.references = references;
         obj.tasks = BuildTasks(obj,N);
      end    

%       function SetAlpha(obj,alpha)
%          obj.alpha = alpha;
%       end



      function  tau  = Policy(obj,t,q,qd,x,xd,rpy,rpyd)
       % managed combine_rule
      end    
   end
    
end






