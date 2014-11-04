%#TODO set control of input plus error sginaling

function SetTraj(obj,iter)

  if(strcmp(obj.type,'joint'))
  % write joint block
  elseif(strcmp(obj.type,'cartesian'))

      if(strcmp(obj.control_type,'tracking'))
          if(strcmp(obj.traj,'point-point_quintic'))

          elseif(strcmp(obj.traj,'point-point_trapezoidal'))

          else
            error('is not possible to use circular with pose + pos for tracking');
          end 
      elseif(strcmp(obj.control_type,'regulation'))

          if(strcmp(obj.traj,'point-point_quintic'))

          elseif(strcmp(obj.traj,'point-point_trapezoidal'))

          else
            error('is not possible to use circular with pose + pos for regulation');
          end      
      end 

  elseif(strcmp(obj.type,'cartesian_pos'))
  
      if(strcmp(obj.control_type,'tracking'))
       
          if(strcmp(obj.traj,'point-point_quintic'))

          elseif(strcmp(obj.traj,'point-point_trapezoidal'))

          elseif(strcmp(obj.traj,'circular')) 
            obj.trajectories{iter} = @Circular;
          end 
          
      elseif(strcmp(obj.control_type,'regulation'))

          if(strcmp(obj.traj,'point-point_quintic'))

          elseif(strcmp(obj.traj,'point-point_trapezoidal'))
       
          else
            error('is not possible to use circular with  cartesian_pos for regulation');
          end      
      end 

  elseif(strcmp(obj.type,'cartesian_pose'))    
  % write cartesian_pose block
  end

end
















