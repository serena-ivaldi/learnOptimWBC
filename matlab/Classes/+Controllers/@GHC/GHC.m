classdef  GHC < Controllers.AbstractController
    
   properties
      subchains;       % object that contains the subchain of the robot and the J dot for each subchain;   (maybe i can leave it) 
      references;      % object that contains the reference trajectory for each tasks; 
      alpha;           % cell array of weight function
      constraints      % cell of constraint objects 
      current_chain    % index that define the current robot that i want to move
      Kp               % vector of matrix of proportional gain
      Kd               % vector of matrix of derivative gain
      regularizer      % value of the regularization term for each chain (column vector) 
      epsilon          % parameter used in the computation of the orthogonal basis for 
      delta_t          % sampling rate used during simulation
      current_time     % current time to force stop for long iteration
      torques          %  resulting torque (cell array of matrix)
      torques_time     % all the time istant when i aply a torque.
      display_opt      % display settings display_opt.step display_opt.trajtrack
      log
   end


   methods
      
       function obj = GHC(sub_chains,references,alpha,constraints,Kp,Kd,regularization,epsilon,delta_t,varargin)
         
        
         obj.subchains = sub_chains;
         obj.references = references;
         obj.alpha = alpha;
         obj.constraints = constraints;
         obj.Kp = Kp;
         obj.Kd = Kd;  
         obj.regularizer = regularization;
         obj.epsilon = epsilon;
         obj.delta_t = delta_t;
         obj.torques = cell(obj.subchains.GetNumChains());
         for i = 1:obj.subchains.GetNumChains()
            obj.torques{i} = zeros(obj.subchains.GetNumLinks(i),1);  %tau(n_of_total_joint on the chain x 1)
         end
         obj.torques_time = cell(obj.subchains.GetNumChains());
         for i = 1 :obj.subchains.GetNumChains()
            obj.torques_time{i} = [];
         end
         obj.current_time = [];
         % default settings for smoothing and trajectory tracking display (desidered position) 
         obj.display_opt.fixed_step = false;
         obj.display_opt.step = 0.00000001;
         obj.display_opt.trajtrack = false;

         
       end    
      
      function SaveTime(obj,ind_subchain,time)
         obj.torques_time{ind_subchain} = [obj.torques_time{ind_subchain}(:,:),time];
      end

      function SaveTau(obj,ind_subchain,tau)
         obj.torques{ind_subchain} = [obj.torques{ind_subchain}(:,:),tau];   
      end
      
      function CleanTau(obj)
          for i = 1 :obj.subchains.GetNumChains()
            obj.torques{i} = [];
          end
      end
      
      function CleanTime(obj)
         for i = 1 :obj.subchains.GetNumChains()
            obj.torques_time{i} = [];
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
      
      
      function  final_tau  = Policy(obj,t,q,qd,Fc)
          %DEBUG
          %t
%           q
%           qd
          %--- 
         
          % active robot 
          cur_bot = obj.GetActiveBot;
          % current chain index
          i = obj.GetCurRobotIndex;
          DOF = cur_bot.n;
          n_of_task = obj.subchains.GetNumTasks(i);
          % the dynamic computation between controller and simulator has
          % to be different
          M = cur_bot.inertia(q);
          F = cur_bot.coriolis(q,qd)*qd' + cur_bot.gravload(q)';
          
          
          % compute the objective function
          % cp = control points are disposed per columns
          [H,f,J_list,cp]=obj.ObjectiveFunction(DOF,i,t,q,qd);
          
          % compute the projector
          [projector_list]=obj.ComputeGeneralizedProjector(i,J_list,t);
          
          % compute matrix for equality constraints
          [Aeq,beq] = obj.EqualityConstraints(M,F,DOF,projector_list);
          
           % compute matrix for disequality constraints 
          [A,b] = obj.DisequalityConstraints(DOF,n_of_task,obj.delta_t,J_list,projector_list,q,qd,cp);
          
          % result
          options = optimset('Display','off','Algorithm','interior-point-convex','TolCon',0.0000000000001); % if i remove display off i see if the optimization problem is solved for each step 
          x=quadprog(H,f,A,b,Aeq,beq,[],[],[],options);
%           n = size(H,1);
%           cvx_begin quiet
%              variable x(n)
%              minimize ( (1/2)*quad_form(x,H) + f*x)
%              Aeq*x == beq;
%              A*x <=  b;
%           cvx_end

          
          final_tau = x(1:DOF,1);
          
          obj.SaveTau(i,final_tau) 
          obj.SaveTime(i,t);  
      end
      
       %% all the function from this point DO NOT SUPPORT multichain structure (this part work only with RBF)
      % in this function i update the value of the alpha function giving
      % new set of parameters
      
      % the implicit rule with repellers is that first i update the rbf
      % functions for the task and after i update the alpha function
      % for repellers 
      function UpdateParameters(obj,parameters)
       disp('im in update parameters')   
         for i=1:size(obj.alpha,1) 
             index = 1;
             for j=1:size(obj.alpha,2)  
                 n_param = obj.alpha{i,j}.GetParamNum();
                 app_param = parameters(index:index+n_param - 1);
                 obj.alpha{i,j}.ComputeNumValue(app_param')
                 index = index+n_param;
             end
         end
      end
      
       
      function n_param=GetTotalParamNum(obj)
          
          n_param = 0;
          for i=1:1:size(obj.alpha,1) 
             for j=1:size(obj.alpha,2) 
                 n_param = n_param + obj.alpha{i,j}.GetParamNum();
             end
          end
      end
      

      
      
   end
    
end






