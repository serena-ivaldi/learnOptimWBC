classdef  UF < Controllers.AbstractController
    
   properties
      subchains;       % object that contains the subchain of the robot and the J dot for each subchain;   (maybe i can leave it) 
      references;      % object that contains the reference trajectory for each tasks; 
      alpha;           % cell array of weight function
      metric;          % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q)) 
      ground_truth     % if true for computing the position and velocity of the end effector i will use the non perturbed model 
      Kp               % vector of matrix of proportional gain
      Kd               % vector of matrix of derivative gain
      combine_rule     % projector or sum 
      torques          %  resulting torque (vector of matrix)
      display_opt      % display settings display_opt.step display_opt.trajtrack
   end


   methods
      
       function obj = UF(sub_chains,references,alpha,metric,ground_truth,Kp,Kd,combine_rule,varargin)
         
         obj.subchains = sub_chains;
         obj.references = references;
         obj.alpha = alpha;
         obj.metric = metric;
         obj.ground_truth = ground_truth;
         obj.Kp = Kp;
         obj.Kd = Kd;
         obj.combine_rule = combine_rule;
         obj.torques = cell(obj.references.GetNumTasks());
         for i = 1:obj.references.GetNumTasks()
            obj.torques{i} = zeros(obj.subchains.n,1);  %tau(n_of_total_joint on the chain x 1 x n_of_task)
         end
         % default settings for smoothing and trajectory tracking display (desidered position) 
         obj.display_opt.fixed_step = false;
         obj.display_opt.step = 0.00000001;
         obj.display_opt.trajtrack = false;
         % settings for smoothing and trajectory tracking display (reference position)
         if (nargin > 7)
            disp('sono qui');
            disp_opt = varargin{1};
            obj.display_opt.step =disp_opt.step;
            obj.display_opt.trajtrack = disp_opt.trajtrack;   
         end
         
      end    

      function SaveTau(obj,index,tau)
         obj.torques{index} = [obj.torques{index}(:,:),tau];   
      end
      
      function CleanTau(obj)
          for i = 1 :obj.references.GetNumTasks()
            obj.torques{i} = [];
          end
      end
      
      % in this function i update the value of the alpha function giving
      % new set of parameters
      function UpdateParameters(obj,parameters)
          index = 1;
          for i=1:obj.references.GetNumTasks() 
              n_param = obj.alpha{i}.GetParamNum();
              app_param = parameters(index:index+n_param - 1);
              obj.alpha{i}.ComputeNumValue(app_param')
              index = index+n_param;
          end
      end

      function  final_tau  = Policy(obj,t,q,qd)
         
        if(strcmp(obj.combine_rule,'sum')) 
           
           %#TODO instead of using RNE maybe is better to precompute the dynamics term using the symbolic routines inside the toolboox
           %symbolic and then trasform them as function  
           
           % number of links of the complete kinematic chain
           n = obj.subchains.n;
           
           % the dynamic computation between controller and simulator has
           % to be different
           
           %   compute current manipulator inertia
           %   torques resulting from unit acceleration of each joint with
           %   no gravity.
           M = rne(obj.subchains.dyn_model, ones(n,1)*q, zeros(n,n), eye(n), [0;0;0]);
           %    compute gravity and coriolis torque
           %    torques resulting from zero acceleration at given velocity &
           %    with gravity acting.
           F = rne(obj.subchains.dyn_model, q, qd, zeros(1,n)); 
           
           final_tau = zeros(n,1);
           for index =1:obj.references.GetNumTasks()
               tau = ComputeTorqueSum(obj,index,M,F,t,q,qd);
               obj.SaveTau(index,tau);
               final_tau = final_tau + obj.alpha{index}.GetValue(t)*tau;  
               
           end
         
        elseif(strcmp(obj.combine_rule,'projector'))
        %#TODO   
        end   
       % add combine_rule
      end  
      
      function n_param=GetTotalParamNum(obj)
         
          n_param = 0;
          for i=1:size(obj.alpha,2) 
              n_param = n_param + obj.alpha{i}.GetParamNum();
          end
      end
      
      
   end
    
end






