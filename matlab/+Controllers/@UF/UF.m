classdef  UF < AbstractController
    
   properties
      references;       % object that contains the reference trajectory for each tasks; 
      subchains;        % object that contains the subchain of the robot and the J dot for each subchain;   
      %alpha;           % vector of weight function
      N;                % vector of metric (vector of matrix) 
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



      function  tau  = Policy(obj,M_inv,F,t,q,qd,x,xd,rpy,rpyd)
         
        if(strcmp(obj.combine_rule,'sum')) 
           
           for index =1:obj.references.GetNumTasks()
               %#TODO inserire gli alpha
               tau = tau + ComputeTorqueSum(obj,index,M_inv,F,t,q,qd,x,xd,rpy,rpyd);
               
           end
         
        elseif(strcmp(obj.combine_rule,'projector'))
           
        end   
       % managed combine_rule
      end    
   end
    
end






