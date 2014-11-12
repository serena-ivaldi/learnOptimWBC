classdef  References < handle
    
   properties
      subchain;     % handle to subchain (subchain contiene il numero dei task)
      type;         % cartesian_x,cartesian_rpy, joint vector 
      control_type; % tracking,regulation vector
      traj;         % circular, point-point_quintic, point-point_trapezoidal vector
      parameters;   % vector of parameters that define the properties of every trajectories (both functional and sampled)
      mask;         % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      type_of_traj; % sampled func
      trajectories; % cell array with the sampling of the trajectory with position velocity and desired acceleration  
                    % in case of sampled trajectory i build up a struct with four fields "sample_x" "sample_xd" "sample_xdd" (row matrix) contains the value of the  trajectory 
                    %and "time" (row vector) that contains the sampling time
                    
   end
       
    
   methods
      %#TODO add control of the input
      function obj = References(subchain,type,control_type,traj,parameters,mask,type_of_traj,varargin) % i can specify through varargin the time duration of the sampled trajectories 
         
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
         
         if(getnameidx({ 'sampled', 'func'} , type_of_traj) ~= 0)
            obj.type_of_traj = type_of_traj;
         end
         
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
            %obj.SetTypeOfTraj(i)
          end
          
      end
      
      
      function [p,pd,pdd]=GetTraj(obj,index,t)
         
         if(strcmp(obj.type_of_traj(index,:),'func')) 
            
            [p,pd,pdd]=feval(obj.trajectories{index},t,obj.type_of_traj,obj.parameters);    
         
         elseif(strcmp(obj.type_of_traj(index,:),'sampled'))
            
            % index to the current value
            [~,ind] = min(abs(obj.trajectories{index}.time-t));
            %DEBUG
            cur_time = f(ind); % Finds first one only! 
            disp(cur_time);
            %--
            p  = obj.trajectories{index}.sample_x(:,ind);
            pd = obj.trajectories{index}.sample_xd(:,ind);
            pdd= obj.trajectories{index}.sample_xdd(:,ind);
         
         end
         
      end
      
      
      %function that set the flag on the kind of trajectories
      function SetTypeOfTraj(obj,index)
         
         if(isa(obj.trajectories{index},'function_handle'))
         
            obj.type_of_traj = 'func';  
         
         elseif(isa(obj.trajectories{index},'numeric') )
         
            obj.type_of_traj = 'sampled';
         
         end   
      end
     
      
      
      
   end
    
end