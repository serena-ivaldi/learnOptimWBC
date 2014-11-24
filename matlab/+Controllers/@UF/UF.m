classdef  UF < Controllers.AbstractController
    
   properties
      subchains;       % object that contains the subchain of the robot and the J dot for each subchain;   (maybe i can leave it) 
      references;      % object that contains the reference trajectory for each tasks; 
      alpha;           % cell array of weight function
      metric;          % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q)) 
      current_chain    % index that define the current robot that i want to move
      %ground_truth     % if true for computing the position and velocity of the end effector i will use the non perturbed model 
      Kp               % vector of matrix of proportional gain
      Kd               % vector of matrix of derivative gain
      combine_rule     % projector or sum 
      torques          %  resulting torque (cell array of matrix)
      display_opt      % display settings display_opt.step display_opt.trajtrack
   end


   methods
      
       function obj = UF(sub_chains,references,alpha,metric,Kp,Kd,combine_rule,varargin)
         
         obj.subchains = sub_chains;
         obj.references = references;
         obj.alpha = alpha;
         obj.metric = metric;
         obj.Kp = Kp;
         obj.Kd = Kd;
         obj.combine_rule = combine_rule;
         obj.torques = cell(obj.subchains.GetNumChains());
         for i = 1:obj.subchains.GetNumChains()
            obj.torques{i} = zeros(obj.subchains.GetNumLinks(i),1);  %tau(n_of_total_joint on the chain x 1)
         end
         % default settings for smoothing and trajectory tracking display (desidered position) 
         obj.display_opt.fixed_step = false;
         obj.display_opt.step = 0.00000001;
         obj.display_opt.trajtrack = false;
         % settings for smoothing and trajectory tracking display (reference position)
         if (nargin > 7)
            disp_opt = varargin{1};
            obj.display_opt.step =disp_opt.step;
            obj.display_opt.trajtrack = disp_opt.trajtrack;   
         end
         
      end    

      function SaveTau(obj,ind_subchain,tau)
         obj.torques{ind_subchain} = [obj.torques{ind_subchain}(:,:),tau];   
      end
      
      function CleanTau(obj)
          for i = 1 :obj.subchains.GetNumChains()
            obj.torques{i} = [];
          end
      end
      
      function SetCurRobotIndex(obj,index_chain)
          obj.current_chain = index_chain;
      end
      
      function i = GetCurRobotIndex(obj)
          i = obj.current_chain;
      end
      
      function bot = GetActiveBot(obj)
          bot = obj.subchains.GetCurRobot(obj.current_chain);
      end
      
      function bot = GetActiveBotVis(obj)
          bot = obj.subchains.GetCurRobotVis(obj.current_chain);
      end
      
      
      function  final_tau  = Policy(obj,t,q,qd)
         
           if(strcmp(obj.combine_rule,'sum')) 

             % active robot 
             cur_bot = obj.GetActiveBot;
             % current chain index
             i = obj.GetCurRobotIndex;
             % the dynamic computation between controller and simulator has
             % to be different

             M = cur_bot.inertia(q);
             F = cur_bot.coriolis(q,qd)*qd' + cur_bot.gravload(q)';


             %final_tau = zeros(cur_bot.n,1)
             for j =1:obj.subchains.GetNumTasks(i)
                 tau(:,j) = obj.alpha{i,j}.GetValue(t)*obj.ComputeTorqueSum(i,j,M,F,t,q,qd);
                 % here i have to put a subset of functions t that i want to
                 % use to catch data for computing fitness func
                 %final_tau = final_tau + obj.alpha{i,j}.GetValue(t)*tau;  
                 
             end
             final_tau = sum(tau,2);
             obj.SaveTau(i,final_tau)    
              
           elseif(strcmp(obj.combine_rule,'projector'))
           %#TODO % add combine_rule  
           end   
      end
      
      % TO FIX
      % in this function i update the value of the alpha function giving
      % new set of parameters
      function UpdateParameters(obj,parameters)
          
         for i=1:obj.subchains.GetNumChains() 
             index = 1;
             for j=1:obj.subchains.GetNumTasks()  
                 n_param = obj.alpha{i,j}.GetParamNum();
                 app_param = parameters(index:index+n_param - 1);
                 obj.alpha{i,j}.ComputeNumValue(app_param')
                 index = index+n_param;
             end
         end
      end
      
      
      
      % TO FIX i have to see in instance 
      function n_param=GetTotalParamNum(obj,ind_subchain)
         
          n_param = 0;
          for i=1:obj.subchains.GetNumChains()
             for j=1:obj.subchains.GetNumTasks(i) 
                 n_param = n_param + obj.alpha{i,j}.GetParamNum();
             end
          end
      end
      
      
   end
    
end






