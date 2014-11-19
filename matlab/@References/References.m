classdef  References < handle
    
   properties
      subchain;        % handle to subchain (subchain contiene il numero dei task)
      type;            % cartesian_x,cartesian_rpy, joint vector 
      control_type;    % tracking,regulation vector
      traj;            % circular, rectilinear, point-point_quintic, point-point_trapezoidal vector
      geom_parameters; % vector of parameters that define the properties of every trajectories (both functional and sampled)
      time_law;        % exponential,linear,constant,trapezoidal
      time_parameters; % vector of the time parameters of the specified time law
      time_struct      % struct with time_struct.ti time_struct.tf time_struct.step
      mask;            % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      type_of_traj;    % sampled func
      trajectories;    % cell array with the sampling of the trajectory with position velocity and desired acceleration  
                       % in case of sampled trajectory i build up a struct with four fields "p" "pd" "pdd" (row matrix) contains the value of the  trajectory 
                       %and "time" (row vector) that contains the sampling time
                    
   end
       
    
   methods
      %#TODO add control of the input
      function obj = References(subchain,type,control_type,traj,geom_parameters,time_law,time_parameters,time_struct,mask,type_of_traj,varargin) % i can specify through varargin the time duration of the sampled trajectories 
         
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
         if(getnameidx({ 'circular','rectilinear', 'point-point_quintic' , 'point-point_trapezoidal' 'none'} , traj) ~= 0)
            obj.traj = traj;
         end
         
         if(getnameidx({'exponential','linear','constant','trapezoidal' 'none'} , time_law) ~= 0)
            obj.time_law = time_law;
         end
         
         obj.geom_parameters = geom_parameters;
         obj.time_parameters = time_parameters;
         obj.time_struct = time_struct;
         obj.mask = mask;
         
         if(getnameidx({ 'sampled', 'func'} , type_of_traj) ~= 0)
            obj.type_of_traj = type_of_traj;
         end
         
            
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
         
         if(strcmp(obj.control_type{index},'regulation'))
             p  = obj.trajectories{index}.p;
             pd = obj.trajectories{index}.pd;
             pdd= obj.trajectories{index}.pdd; 

         elseif(strcmp(obj.control_type{index},'tracking'))
             
             if(strcmp(obj.type_of_traj{index},'func')) 

                %normtime = NormalizeTime(t,obj.time_struct.ti,obj.time_struct.tf); 

                p=feval(obj.trajectories{index}.p,t);
                pd=feval(obj.trajectories{index}.pd,t);
                pdd=feval(obj.trajectories{index}.pdd,t);


             elseif(strcmp(obj.type_of_traj{index},'sampled'))

                % index to the current value
                [~,ind] = min(abs(obj.trajectories{index}.time-t));
                %DEBUG
                %cur_time = f(ind); % Finds first one only! 
                %disp(cur_time);
                %--
                p  = obj.trajectories{index}.p(:,ind);
                pd = obj.trajectories{index}.pd(:,ind);
                pdd= obj.trajectories{index}.pdd(:,ind);

             end
         end
      end      
      
   end
    
end