%#TODO set control of input plus error sginaling

function SetTraj(obj,iter)

  if(strcmp(obj.type,'joint'))
  % write joint block

  elseif(strcmp(obj.type,'cartesian_x'))
  
      if(strcmp(obj.control_type,'tracking'))
       
          if(strcmp(obj.traj,'point-point_quintic'))

          elseif(strcmp(obj.traj,'point-point_trapezoidal'))

          elseif(strcmp(obj.traj,'circular')) 
              
              s = 0;
              if(strcmp(obj.time_law,'exponential'))
                  disp('sono in exponential')
                  s=Exponential(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law,'linear'))
                  s=Linear(obj.time_struct.tf,obj.time_parameters);
              elseif(strcmp(obj.time_law,'constant'))
                  %s=Constant(obj.time_parameters);
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

          if(strcmp(obj.traj,'point-point_quintic'))

          elseif(strcmp(obj.traj,'point-point_trapezoidal'))
       
          else
            error('is not possible to use circular with cartesian_pos for regulation');
          end      
      end 

  elseif(strcmp(obj.type,'cartesian_rpy'))    
  % write cartesian_pose block
  end

end
















