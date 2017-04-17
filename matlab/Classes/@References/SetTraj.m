%#TODO set control of input plus error signaling
% I control the normalization of time inside the time law (linear circular and trapezoid)

function SetTraj(obj,ind_subchain,ind_task)
     if(strcmp(obj.traj{ind_subchain,ind_task},'rectilinear'))
          s = 0;
          if(strcmp(obj.time_law{ind_subchain,ind_task},'exponential'))
              s=Exponential(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'linear'))
              s=Linear(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'trapezoidal'))
              %s=Trapezoidal(obj.time_parameters);
          end
          [p,pd,pdd,t] = Rectilinear(s,obj.time_struct,obj.geom_parameters{ind_subchain,ind_task},obj.type_of_traj{ind_subchain,ind_task});
          obj.trajectories{ind_subchain,ind_task}.p = p;
          obj.trajectories{ind_subchain,ind_task}.pd = pd;
          obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
          obj.trajectories{ind_subchain,ind_task}.time = t;
          obj.time_law_and_derivatives.s = s;
     elseif(strcmp(obj.traj{ind_subchain,ind_task},'lemniscate')) 
          s = 0;
          if(strcmp(obj.time_law{ind_subchain,ind_task},'exponential'))
              s=Exponential(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'linear'))
              s=Linear(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'trapezoidal'))
              %s=Trapezoidal(obj.time_parameters);
          end
          [p,pd,pdd,t] = Lemniscate(s,obj.time_struct,obj.geom_parameters{ind_subchain,ind_task},obj.type_of_traj{ind_subchain,ind_task});
          obj.trajectories{ind_subchain,ind_task}.p = p;
          obj.trajectories{ind_subchain,ind_task}.pd = pd;
          obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
          obj.trajectories{ind_subchain,ind_task}.time = t;   
          obj.time_law_and_derivatives.s = s;
     elseif(strcmp(obj.traj{ind_subchain,ind_task},'circular')) 
          s = 0;
          if(strcmp(obj.time_law{ind_subchain,ind_task},'exponential'))
              s=Exponential(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'linear'))
              s=Linear(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'trapezoidal'))
              %s=Trapezoidal(obj.time_parameters);
          end
          [p,pd,pdd,t] = Circular(s,obj.time_struct,obj.geom_parameters{ind_subchain,ind_task},obj.type_of_traj{ind_subchain,ind_task});
          obj.trajectories{ind_subchain,ind_task}.p = p;
          obj.trajectories{ind_subchain,ind_task}.pd = pd;
          obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
          obj.trajectories{ind_subchain,ind_task}.time = t;
          obj.time_law_and_derivatives.s = s;
     elseif(strcmp(obj.traj{ind_subchain,ind_task},'elastic')) 
          s = 0;
          if(strcmp(obj.time_law{ind_subchain,ind_task},'exponential'))
              s=Exponential(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'linear'))
              s=Linear(obj.time_struct.tf);
          elseif(strcmp(obj.time_law{ind_subchain,ind_task},'trapezoidal'))
              %s=Trapezoidal(obj.time_parameters);
          end
          [p,pd,pdd,t] = ElasticReference(s,obj.time_struct,obj.geom_parameters{ind_subchain,ind_task},obj.type_of_traj{ind_subchain,ind_task});
          obj.trajectories{ind_subchain,ind_task}.p = p;
          obj.trajectories{ind_subchain,ind_task}.pd = pd;
          obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
          obj.trajectories{ind_subchain,ind_task}.time = t;
          obj.time_law_and_derivatives.s = s;
          % number of basis x number of euclidean dimensions
          obj.parameter_dim{ind_subchain,ind_task} = obj.geom_parameters{ind_subchain,ind_task}(1)*3; % here 3 represents the number of rbf that im using inside geom_param 1 number of basis
     elseif(strcmp(obj.traj{ind_subchain,ind_task},'AdHocBalance'))
          s = 0;
          [p,pd,pdd,t,visualizatio_struct] = AdHocBalanceControllerTrajectory(s,obj.time_struct,obj.geom_parameters{ind_subchain,ind_task},obj.type_of_traj{ind_subchain,ind_task});
          obj.trajectories{ind_subchain,ind_task}.p = p;
          obj.trajectories{ind_subchain,ind_task}.pd = pd;
          obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
          obj.trajectories{ind_subchain,ind_task}.time = t;
          obj.time_law_and_derivatives.s = visualizatio_struct.s;
          obj.time_law_and_derivatives.sd = visualizatio_struct.sd;
          obj.time_law_and_derivatives.sdd = visualizatio_struct.sdd;
          % number of basis x number of euclidean dimensions
          obj.parameter_dim{ind_subchain,ind_task} = obj.geom_parameters{ind_subchain,ind_task}(1)*3; % here 3 represents the number of rbf that im using inside and geom_param 1 number of basis
     elseif(strcmp(obj.traj{ind_subchain,ind_task},'fixed'))
         [p ,pd,pdd,t]=Fixedpoint(obj.geom_parameters{ind_subchain,ind_task});
         obj.trajectories{ind_subchain,ind_task}.p = p;
         obj.trajectories{ind_subchain,ind_task}.pd = pd;
         obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
         obj.trajectories{ind_subchain,ind_task}.time =t;
         % if type_of_traj is sampled the value of p pd and pdd is
         % overwritten with only one value
         if(strcmp(obj.type_of_traj{ind_subchain,ind_task},'sampled'))
             [p,pd,pdd]=GetFuncTraj(obj,ind_subchain,ind_task,0);
             obj.trajectories{ind_subchain,ind_task}.p = p;
             obj.trajectories{ind_subchain,ind_task}.pd = pd;
             obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
         end
     elseif(strcmp(obj.traj{ind_subchain,ind_task},'ball'))
         [p,pd,pdd,t]=FixedBall(obj.geom_parameters{ind_subchain,ind_task});
         obj.trajectories{ind_subchain,ind_task}.p = p;
         obj.trajectories{ind_subchain,ind_task}.pd = pd;
         obj.trajectories{ind_subchain,ind_task}.pdd = pdd;
         obj.trajectories{ind_subchain,ind_task}.time =t;
         obj.parameter_dim{ind_subchain,ind_task} = 3;
     end 
  
     
end

