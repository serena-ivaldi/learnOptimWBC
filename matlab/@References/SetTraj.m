%#TODO set control of input plus error sginaling

function SetTraj(obj,iter)

  if(strcmp(obj.type,'joint'))
  % write joint block

  elseif(strcmp(obj.type,'cartesian_x'))
  
      if(strcmp(obj.control_type,'tracking'))
       
          if(strcmp(obj.traj,'rectilinear'))
              disp('rettilineo')
              s = 0;
              if(strcmp(obj.time_law,'exponential'))
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law,'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law,'trapezoidal'))
                  %s=Trapezoidal(obj.time_parameters);
              end
              
              [p pd pdd t] = Rectilinear(s,obj.time_struct,obj.geom_parameters,obj.type);
              obj.trajectories{iter}.p = p;
              obj.trajectories{iter}.pd = pd;
              obj.trajectories{iter}.pdd = pdd;
              obj.trajectories{iter}.time = t;
              
              
          elseif(strcmp(obj.traj,'circular')) 
              disp('curvilineo')
              s = 0;
              if(strcmp(obj.time_law,'exponential'))
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law,'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law,'trapezoidal'))
                  %s=Trapezoidal(obj.time_parameters);
              end
            
              [p pd pdd t] = Circular(s,obj.time_struct,obj.geom_parameters,obj.type);
              obj.trajectories{iter}.p = p;
              obj.trajectories{iter}.pd = pd;
              obj.trajectories{iter}.pdd = pdd;
              obj.trajectories{iter}.time = t;
          end 
          
      elseif(strcmp(obj.control_type,'regulation'))
         [p pd pdd t]=Fixedpoint(obj.geom_parameters);
         obj.trajectories{iter}.p = p;
         obj.trajectories{iter}.pd = pd;
         obj.trajectories{iter}.pdd = pdd;
         obj.trajectories{iter}.time = t;    
      end 

  elseif(strcmp(obj.type,'cartesian_rpy'))    
  % write cartesian_pose block
  end

end
















