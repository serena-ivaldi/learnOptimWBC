classdef  UF < AbstractController
    
   properties
      references; % object that contains the reference trajectory for each tasks; 
      subchains; % object that contains the subchain of the robot and the J dot for each subchain;   
      alpha;      % vector of weight function
      tasks;      % vector of function handle that describe the current task
      policy = @Policy;  %pointer to the function policy
   end


   methods
      function obj = UF(sub_chains,references,N)
         obj.subchain = sub_chains;
         obj.references = references;
         obj.tasks = BuildTasks(obj,N);
      end    

      function SetAlpha(obj,alpha)
         obj.alpha = alpha;
      end

      function BuildTasks(obj,N)
         disp()
      end


      function  tau  = Policy(obj,t,q,qd,x_cur,xd_cur,pose_cur,posed_cur)

      end    
   end
    
end






