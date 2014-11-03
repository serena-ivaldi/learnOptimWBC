classdef  References < handle
    
   properties
      subchain;     % handle to subchain (subchain contiene il numero dei task)
      type;         % cartesian (pos + pose), cartesian_pos (x y z), cartesian_pose (roll pitch yaw), joint (subchain.n) vector 
      control_type; % tracking,regulation vector
      traj;         % circle, point-point_quintic, point-point_trapezoidal vector
      parameters    % vector of parameters that define the properties of every trajectories
      trajectories; % cell vector with the sampling of the trajectory with position velocity and desired acceleration
      
   end
       
    
   methods
      
      function obj = References(subchain,type,control_type,traj,varargin) % in varargin posso metterci la durata della traiettoria 
         
         if (isa(subchain,'function_handle'))
            obj.subchain = subchain;
         else
            error('first argument must be a function handle')   
         end
         if(any(['cartesian' 'cartesian_pos' 'cartesian_pose' 'joint'] == type))
            obj.type = type;
         end
         if(any(['tracking' 'regulation'] == control_type))
            obj.control_type = control_type;
         end
         if(any([ 'circle', 'point-point_quintic' , 'point-point_trapezoidal'] == traj)
            obj.traj = traj;
         end
          
      end  
      
      function BuildTraj(obj)
          
          if(strcmp(obj.type,'joint'))
          % implementare controlli ai giunti :) 
          elseif(strcmp(obj.type,'cartesian'))
               
              if(strcmp(obj.control_type,'tracking'))
                  if(strcmp(obj.traj,'point-point_quintic'))
                      
                  elseif(strcmp(obj.traj,'point-point_trapezoidal'))
                      
                  else
                    error('is not possible to use circular with pose + pos for tracking');
                    return;
                  end 
              elseif(strcmp(obj.control_type,'regulation'))
                  
                  if(strcmp(obj.traj,'point-point_quintic'))
                      
                  elseif(strcmp(obj.traj,'point-point_trapezoidal'))
                      
                  else
                    error('is not possible to use circular with pose + pos for regulation');
                    return;
                  end      
              end 
               
          elseif(strcmp(obj.type,'cartesian_pos'))
          
          elseif(strcmp(obj.type,'cartesian_pose'))    
          
          end
          
      end
      
      
      
   end
    
end