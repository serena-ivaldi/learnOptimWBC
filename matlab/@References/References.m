classdef  References < handle
    
   properties
      subchain;     % handle to subchain (subchain contiene il numero dei task)
      type;         % cartesian (pos + pose), cartesian_pos (x y z), cartesian_pose (roll pitch yaw), joint (subchain.n) vector 
      control_type; % tracking,regulation vector
      traj;         % circular, point-point_quintic, point-point_trapezoidal vector
      fun_or_sample;% if true (when it is possible) i specify the trajectory with the function handle if false i use a set of samples
      parameters    % vector of parameters that define the properties of every trajectories (both functional and sampled)
      trajectories; % cell array with the sampling of the trajectory with position velocity and desired acceleration
      
   end
       
    
   methods
      %#TODO add control of the input
      function obj = References(subchain,type,control_type,traj,parameters,varargin) % i can specify through varargin the time duration of the sampled trajectories 
         
         if (isobject(subchain))
            obj.subchain = subchain;
         else
            error('first argument must be a subchain object')   
         end
         if(getnameidx({'cartesian' 'cartesian_pos' 'cartesian_pose' 'joint'} , type) ~= 0 )
            obj.type = type;
         end
         if(getnameidx({'tracking' 'regulation'} , control_type) ~= 0)
            obj.control_type = control_type;
         end
         if(getnameidx({ 'circular', 'point-point_quintic' , 'point-point_trapezoidal'} , traj) ~= 0)
            obj.traj = traj;
         end
         
         obj.parameters = parameters;
         
         % here i have to specify all the optional properties
         %opt.n = 50;
         %[opt,arglist] = tb_optparse(opt, varargin);
         
          
      end  
      
      

      
      function n=GetNumTasks(obj)
          n=size(obj.subchain.target_link,2);
      end  

      
      function BuildTrajs(obj)
          
          for i = 1:obj.GetNumTasks
            obj.SetTraj(i)    
          end
      end
      
      
      function [p,pd,pdd]=GetTraj(obj,index,t)
        [p,pd,pdd]=feval(obj.trajectories{index},t,obj.parameters);       
      end
      
      
       
      
      
     
      
      
   end
    
end