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
      max_time         % maximum time simulation allowed
      current_time     % current time to force stop for long iteration
      torques          %  resulting torque (cell array of matrix)
      display_opt      % display settings display_opt.step display_opt.trajtrack
   end


   methods
      
       function obj = GHC(sub_chains,references,alpha,constraints,Kp,Kd,regularization,epsilon,delta_t,max_time,varargin)
         
        
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
         obj.max_time = max_time;
         obj.current_time = [];
         % default settings for smoothing and trajectory tracking display (desidered position) 
         obj.display_opt.fixed_step = false;
         obj.display_opt.step = 0.00000001;
         obj.display_opt.trajtrack = false;

         
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
          [A,b] = obj.DisequalityConstraints(DOF,n_of_task,obj.delta_t,J_list,projector_list,qd,cp);
          
          % result
          x=quadprog(H,f,A,b,Aeq,beq);
          final_tau = x(1:DOF); 
          
          obj.SaveTau(i,final_tau) 
      end
      
      %empty only for fullfil the requiriments of the interface 
      function GetTotalParamNum(obj)

      end

      
      
   end
    
end






