classdef  References < handle
    
   properties
      subchain;     % handle to subchain (subchain contiene il numero dei task)
      type;         % cartesian_x cartesian_rpy, joint vector 
      control_type; % tracking,regulation vector
      traj;         % circular, point-point_quintic, point-point_trapezoidal vector
      parameters;   % vector of parameters that define the properties of every trajectories (both functional and sampled)
      mask;         % vector of vector(3) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      type_of_traj; % sampled function; (set internally)
      trajectories; % cell array with the sampling of the trajectory with position velocity and desired acceleration  
   end
       
    
   methods
      %#TODO add control of the input
      function obj = References(subchain,type,control_type,traj,parameters,mask,varargin) % i can specify through varargin the time duration of the sampled trajectories 
         
         if (isobject(subchain))
            obj.subchain = subchain;
         else
            error('first argument must be a subchain object')   
         end
         if(getnameidx({'joint' 'cartesian_x' 'cartesian_rpy'} , type) ~= 0 )
            obj.type = type;
         end
         if(getnameidx({'tracking' 'regulation'} , control_type) ~= 0)
            obj.control_type = control_type;
         end
         if(getnameidx({ 'circular', 'point-point_quintic' , 'point-point_trapezoidal'} , traj) ~= 0)
            obj.traj = traj;
         end
         
         obj.parameters = parameters;
         obj.mask = mask;
         
         %#TODO
         %here i have to specify all the optional properties
         %opt.n = 50;
         %[opt,arglist] = tb_optparse(opt, varargin);
            
      end  
      
         
      function n=GetNumTasks(obj)
          n=size(obj.subchain.target_link,2);
      end  

      
      function BuildTrajs(obj)
          
          for i = 1:obj.GetNumTasks
            obj.SetTraj(i)
            obj.SetTypeOfTraj(i)%#TODO
          end
          
      end
      
      
      function [p,pd,pdd]=GetTraj(obj,index,t)
        [p,pd,pdd]=feval(obj.trajectories{index},t,obj.parameters);       
      end
      
      
      %#TODO function that set the flag 
      function SetTypeOfTraj(index)
      end
      %#TODO function that resolve the sample to match the current value of
      % trajectory respect of time 
      function [k kd kdd]=GetValueTraj(obj,t)
      end
      
      
     
      
      
   end
    
end