classdef  UF < Controllers.AbstractController
    
   properties
      subchains;       % object that contains the subchain of the robot and the J dot for each subchain;   (maybe i can leave it) 
      references;      % object that contains the reference trajectory for each tasks; 
      alpha;           % cell array of weight function
      metric;          % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q)) 
      %ground_truth     % if true for computing the position and velocity of the end effector i will use the non perturbed model 
      Kp               % vector of matrix of proportional gain
      Kd               % vector of matrix of derivative gain
      combine_rule     % projector or sum 
      torques          %  resulting torque (vector of matrix)
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
            obj.torques{i} = zeros(obj.subchains.GetNumLinks(i),1,obj.subchains.GetNumTasks(i));  %tau(n_of_total_joint on the chain x 1 x n_of_task)
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

      function SaveTau(obj,ind_subchain,ind_task,tau)
         obj.torques{ind_subchain} = [obj.torques{index}(:,:,ind_task),tau];   
      end
      
      function CleanTau(obj)
          for i = 1 :obj.subchains.GetNumTasks()
            obj.torques{i} = [];
          end
      end
      
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

      function  final_tau  = Policy(obj,t,q,qd)
         
           if(strcmp(obj.combine_rule,'sum')) 

               for i = 1:obj.subchains.GetNumChains()

                 % number of links of the complete kinematic chain
                 n = obj.subchains.GetNumLinks(i);

                 % the dynamic computation between controller and simulator has
                 % to be different

                 M = obj.subchains.sub_chains{i}.inertia(q);
                 F = obj.subchains.sub_chains{i}.coriolis(q,qd)*qd' + obj.subchains.sub_chains{i}.gravload(q);


                 final_tau = zeros(n,1);
                 for j =1:obj.subchains.GetNumTasks(i)
                     tau = obj.ComputeTorqueSum(i,j,M,F,t,q,qd);
                     % here i have to put a subset of functions t that i want to
                     % use to catch data for computing fitness func
                     obj.SaveTau(i,j,tau);
                     final_tau = final_tau + obj.alpha{i,j}.GetValue(t)*tau;  
                 end
                 
               end
              
           elseif(strcmp(obj.combine_rule,'projector'))
           %#TODO % add combine_rule  
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






