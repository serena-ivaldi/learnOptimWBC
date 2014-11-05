classdef  UF < AbstractController
    
   properties
      references;       % object that contains the reference trajectory for each tasks; 
      subchains;        % object that contains the subchain of the robot and the J dot for each subchain;   
      %alpha;           % vector of weight function
      metric;           % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q)) 
      ground_truth      % if true for computing the position and velocity of the end effector i will use the non perturbed model 
      K_p               % vector of matrix of proportional gain
      K_d               % vector of matrix of derivative gain
      combine_rule      % projector or sum 
   end


   methods
      function obj = UF(sub_chains,references,metric,ground_truth,K_p,K_d,combine_rule)
         obj.subchain = sub_chains;
         obj.references = references;
         obj.metric = metric;
         obj.ground_truth = ground_truth;
         obj.K_p = K_p;
         obj.K_d = K_d;
         obj.combine_rule = combine_rule;
      end    

%       function SetAlpha(obj,alpha)
%          obj.alpha = alpha;
%       end



      function  tau  = Policy(obj,t,q,qd)
         
        if(strcmp(obj.combine_rule,'sum')) 
           
           %#TODO instead of using RNE maybe is better to precompute the dynamics term using the symbolic routines inside the toolboox
           
           %symbolic and then trasform them as function  
           
           % the dynamic computation between controller and simulator has
           % to be different
           
           % compute current manipulator inertia
           %   torques resulting from unit acceleration of each joint with
           %   no gravity.
           M = rne(obj, ones(n,1)*q, zeros(n,n), eye(n), [0;0;0]);
           M_inv = inv(M);
           % compute gravity and coriolis torque
           %    torques resulting from zero acceleration at given velocity &
           %    with gravity acting.
           F = rne(robot, q, qd, zeros(1,n)); 
            
           for index =1:obj.references.GetNumTasks()
               %#TODO inserire gli alpha
               tau = tau + ComputeTorqueSum(obj,index,M,M_inv,F,t,q,qd);  
           end
         
        elseif(strcmp(obj.combine_rule,'projector'))
           
        end   
       % managed combine_rule
      end    
   end
    
end






