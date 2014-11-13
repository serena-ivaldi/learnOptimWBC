%#TODO set control of input plus error sginaling

function SetTraj(obj,iter)

  if(strcmp(obj.type,'joint'))
  % write joint block

  elseif(strcmp(obj.type{iter},'cartesian_x'))
  
      if(strcmp(obj.control_type{iter},'tracking'))
       
          if(strcmp(obj.traj{iter},'rectilinear'))
              s = 0;
              if(strcmp(obj.time_law{iter},'exponential'))
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'trapezoidal'))
                  %s=Trapezoidal(obj.time_parameters);
              end
              
              [p pd pdd t] = Rectilinear(s,obj.time_struct,obj.geom_parameters{iter},obj.type);
              obj.trajectories{iter}.p = p;
              obj.trajectories{iter}.pd = pd;
              obj.trajectories{iter}.pdd = pdd;
              obj.trajectories{iter}.time = t;
              
              
          elseif(strcmp(obj.traj{iter},'circular')) 
              s = 0;
              if(strcmp(obj.time_law{iter},'exponential'))
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'trapezoidal'))
                  %s=Trapezoidal(obj.time_parameters);
              end
            
              [p pd pdd t] = Circular(s,obj.time_struct,obj.geom_parameters{iter},obj.type);
              obj.trajectories{iter}.p = p;
              obj.trajectories{iter}.pd = pd;
              obj.trajectories{iter}.pdd = pdd;
              obj.trajectories{iter}.time = t;
          end 
          
      elseif(strcmp(obj.control_type{iter},'regulation'))
         [p pd pdd t]=Fixedpoint(obj.geom_parameters{iter});
         obj.trajectories{iter}.p = p;
         obj.trajectories{iter}.pd = pd;
         obj.trajectories{iter}.pdd = pdd;
         obj.trajectories{iter}.time = t;    
      end 

  elseif(strcmp(obj.type{iter},'cartesian_rpy'))  
      
      if(strcmp(obj.control_type{iter},'tracking'))
       
          if(strcmp(obj.traj{iter},'rectilinear'))
              s = 0;
              if(strcmp(obj.time_law{iter},'exponential'))
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'trapezoidal'))
                  %s=Trapezoidal(obj.time_parameters);
              end
              
              [p pd pdd t] = Rectilinear(s,obj.time_struct,obj.geom_parameters{iter},obj.type);
              obj.trajectories{iter}.p = p;
              obj.trajectories{iter}.pd = pd;
              obj.trajectories{iter}.pdd = pdd;
              obj.trajectories{iter}.time = t;
              
              
          elseif(strcmp(obj.traj{iter},'circular')) 
              s = 0;
              if(strcmp(obj.time_law{iter},'exponential'))
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law{iter},'trapezoidal'))
                  %s=Trapezoidal(obj.time_parameters);
              end
            
              [p pd pdd t] = Circular(s,obj.time_struct,obj.geom_parameters{iter},obj.type);
              obj.trajectories{iter}.p = p;
              obj.trajectories{iter}.pd = pd;
              obj.trajectories{iter}.pdd = pdd;
              obj.trajectories{iter}.time = t;
          end 
          
      elseif(strcmp(obj.control_type{iter},'regulation'))
         [p pd pdd t]=Fixedpoint(obj.geom_parameters{iter});
         obj.trajectories{iter}.p = p;
         obj.trajectories{iter}.pd = pd;
         obj.trajectories{iter}.pdd = pdd;
         obj.trajectories{iter}.time = t;    
      end
  
  end

end
















