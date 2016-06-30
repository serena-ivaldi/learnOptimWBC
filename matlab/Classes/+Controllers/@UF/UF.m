classdef  UF < Controllers.AbstractController
    
   properties
      subchains        % object that contains the subchain of the robot and the J dot for each subchain  (maybe i can leave it) 
      references       % object that contains the reference trajectory for each primary tasks 
      Secondary_refs   % object that contains the reference trajecotry for secondary tasks when specified
      alpha            % cell array of weight function
      repellers        % object of repellers
      metric           % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q)) 
      current_chain    % index that define the current robot that i want to move
      %ground_truth    % if true for computing the position and velocity of the end effector i will use the non perturbed model 
      Param            % cell array of matrix that are contains to kind of object Param{i,j}.Kp, Param{i,j}.Kd  and Param{i,j}.M, obj.Param{i,j}.D, obj.Param{i,j}.P 
      Param_secondary  % cell array of matrix that are contains to kind of object Param{i,j}.Kp, Param{i,j}.Kd  and Param{i,j}.M, obj.Param{i,j}.D, obj.Param{i,j}.P 
      combine_rule     % projector or sum 
      regularizer      % this term transform the UF in a regularized UF if it is different from zero. it is a cell array of vector each vector has as many entry as the number of task for the current chain vector
      torque_func      % in this vector i put the handle to the function that i want to use: ComputeRegularizedTorqueSum(...) ComputeTorqueSum(...)
      current_time     % current time to force stop for long iteration
      torques          %  resulting torque (cell array of matrix)
      torques_time     % all the time istant when i aply a torque.
      display_opt      % display settings display_opt.step display_opt.trajtrack
   end


   methods
      
       function obj = UF(sub_chains,references,Secondary_refs,alpha,repellers,metric,Param,Param_secondary,combine_rule,regularization,varargin)
         obj.subchains = sub_chains;
         obj.references = references;
         obj.Secondary_refs = Secondary_refs;
         obj.alpha = alpha;
         obj.repellers = repellers;
         obj.metric = metric;
         obj.Param = Param;
         obj.Param_secondary = Param_secondary;
         obj.combine_rule = combine_rule;
         % in this way i can use a generic long vector inside
         % RuntimeVariable than here i take what i need
         for i = 1:obj.subchains.GetNumChains()
             for j=1:obj.subchains.GetNumTasks(i)
                 app_vector(j) = regularization{i}(1,j);
                 if( app_vector(j) == 0)
                    obj.torque_func{i,j} = @(ind_subchain,ind_task,M,F,t,q,qd,Fc)obj.ComputeTorqueSum(ind_subchain,ind_task,M,F,t,q,qd,Fc);
                 else
                    obj.torque_func{i,j} = @(ind_subchain,ind_task,M,F,t,q,qd,Fc)obj.ComputeRegularizedTorqueSum(ind_subchain,ind_task,M,F,t,q,qd,Fc);
                 end
             end
             obj.regularizer{i}=app_vector;
         end
         
         % initialize torque and torque time
%          obj.torques = cell(obj.subchains.GetNumChains());
%          for i = 1:obj.subchains.GetNumChains()
%             obj.torques{i} = [];  %tau(n_of_total_joint on the chain x 1)
%          end
%          obj.torques_time = cell(obj.subchains.GetNumChains());
%          for i = 1 :obj.subchains.GetNumChains()
%             obj.torques_time{i} = [];
%          end
         obj.current_time = [];
         % default settings for smoothing and trajectory tracking display (desidered position) 
         obj.display_opt.fixed_step = false;
         obj.display_opt.step = 0.00000001;
         obj.display_opt.trajtrack = false;
         % settings for smoothing and trajectory tracking display (reference position)
%          if (nargin > 7)
%             disp_opt = varargin{1};
%             obj.display_opt.step =disp_opt.step;
%             obj.display_opt.trajtrack = disp_opt.trajtrack;   
%          end
         
       end    
      % i do  not need cell object because the time is unique
      function SaveTime(obj,ind_subchain,time)
         obj.torques_time = [obj.torques_time,time];
      end
       
      % i do not need cell object because the simulated model is a unique
      % big system of differential equations
      function SaveTau(obj,ind_subchain,tau)
         obj.torques = [obj.torques,tau];   
      end
      
      function CleanTau(obj)
          for i = 1 :obj.subchains.GetNumChains()
            obj.torques = [];
          end
      end
      
      function CleanTime(obj)
         for i = 1 :obj.subchains.GetNumChains()
            obj.torques_time = [];
         end
      end
      %% TODO
      % this management of multiple chain has to be changed in favor of a 
      % a unique system to refer to to compute the dynamics component of
      % the robot
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
      %% end
      
      function WS = GetWholeSystem(obj)
          WS = obj.subchains.whole_system;
      end
      
      % get pointer to the complete dynamic object
      
      function  final_tau  = Policy(obj,t,q,qd,Fc,Jc_t)
          
          %DEBUG
          %t
          %q
          %qd
          %---
          
         
          % the dynamic computation between controller and simulator has
          % to be different
          %% provisory structure 
          new = false;
          if(~new) 
              % active robot 
              cur_bot = obj.GetActiveBot;
              % current chain index
              %i = obj.GetCurRobotIndex;
              DOF = cur_bot.n; % is used in the projector
              M = cur_bot.inertia(q);
              F = cur_bot.coriolis(q,qd)*qd' + cur_bot.gravload(q)' - Jc_t*Fc;
          else      
              M = obj.subchains.GetM(q);
              % i include the external forces inside F
              F = obj.subchains.GetF(q,qd,Fc,Jc_t);
              [M,F] = obj.subchains.RemoveFloatingBase(M,F,7);
          end
          % controller 
          if(strcmp(obj.combine_rule,'sum')) 
             count = 1;
             for i = 1:obj.subchains.GetNumChains();
                 for j = 1:obj.subchains.GetNumTasks(i)
                     tau=obj.torque_func{i,j}(i,j,M,F,t,q,qd,Fc);
                     app_tau(:,count) = obj.alpha{i,j}.GetValue(t)*tau;
                     count = count + 1;
                 end
             end
             
             final_tau = sum(app_tau,2);
             final_tau = final_tau + F; 
             obj.SaveTau(i,final_tau); 
             obj.SaveTime(i,t);
             
           elseif(strcmp(obj.combine_rule,'projector'))   
             count = 1;
             for i = 1:obj.subchains.GetNumChains();
                 for j = 1:obj.subchains.GetNumTasks(i)
                     tau=obj.torque_func{i,j}(i,j,M,F,t,q,qd,Fc);
                     app_tau(:,j) = obj.alpha{i,j}.GetValue(t)*tau;
                     count = count + 1;
                 end
             end
             
             final_tau = sum(app_tau,2);
             % compute the projector in the null space of repulsor (number of repulsor) 
             for j = 1:obj.repellers.GetNumTasks(i)
               obj.repellers.SetJacob(cur_bot,q,qd,i,j)  
             end                                
             N = obj.repellers.ComputeProjector(i,DOF,obj.subchains.GetNumTasks(i),obj.alpha,t);
             final_tau = ((M*N)/M)*(final_tau + F);
            
             obj.SaveTau(i,final_tau) 
             obj.SaveTime(i,t);  
           end   
      end
      
      %% all the function from this point DO NOT SUPPORT multichain structure (this part work only with RBF)
      % in this function i update the value of the alpha function giving
      % new set of parameters
      
      % the implicit rule with repellers is that first i update the rbf
      % functions for the task and after i update the alpha function
      % for repellers 
      % and then i update the parameter that govern the reference
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
         % update parameters of the references (if there are some)
         for i=1:size(obj.references.parameter_dim,1) 
             for j=1:size(obj.references.parameter_dim,2)  
                 n_param = obj.references.parameter_dim{i,j};
                 app_param = parameters(index:index+n_param - 1);
                 if(n_param>0)
                     obj.references.cur_param_set{i,j} = (app_param');
                     index = index+n_param;
                 else
                     obj.references.cur_param_set{i,j} = app_param;
                 end
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
          for i=1:1:size(obj.references.parameter_dim,1) 
             n_param = n_param + obj.references.GetNumParam(i);
          end
      end
      
      
   end
    
end






