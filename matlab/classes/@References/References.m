classdef  References < handle
    
   properties
      target_link      %  vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
      type;            % cartesian_x,cartesian_rpy, joint vector 
      control_type;    % tracking,regulation vector
      traj;            % circular, rectilinear, point-point_quintic, point-point_trapezoidal vector
      geom_parameters; % vector of parameters that define the properties of every trajectories (both functional and sampled)
      time_law;        % exponential,linear,constant,trapezoidal
      %time_parameters; % vector of the time parameters of the specified time law
      time_struct      % struct with time_struct.ti time_struct.tf time_struct.step
      mask;            % vector of vector(3 or DOF) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      type_of_traj;    % sampled func
      trajectories;    % cell array with the sampling of the trajectory with position velocity and desired acceleration  
                       % in case of sampled trajectory i build up a struct with four fields "p" "pd" "pdd" (row matrix) contains the value of the  trajectory 
                       %and "time" (row vector) that contains the sampling time
                    
   end
       
    
   methods
      %#TODO add control of the input
      function obj = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,mask,type_of_traj,varargin) % i can specify through varargin the time duration of the sampled trajectories 
          
         obj.target_link = target_link;
       
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
         obj.time_struct = time_struct;
         obj.mask = mask;
         
         if(getnameidx({ 'sampled', 'func'} , type_of_traj) ~= 0)
            obj.type_of_traj = type_of_traj;
         end
         
            
      end  
      
     

      
      function BuildTrajs(obj)
         
          for i = 1:size(obj.target_link,1)
             for j = 1:size(obj.target_link{i},2)
               obj.SetTraj(i,j)
             end
          end
          
      end
      
      function n = GetDimTask(obj,ind_subchain,ind_task)
          n = nnz(obj.mask{ind_subchain,ind_task});
      end
      
      function [p,pd,pdd]=GetTraj(obj,ind_subchain,ind_task,t)
         
         if(strcmp(obj.control_type{ind_subchain,ind_task},'regulation'))
             p  = obj.trajectories{ind_subchain,ind_task}.p;
             pd = obj.trajectories{ind_subchain,ind_task}.pd;
             pdd= obj.trajectories{ind_subchain,ind_task}.pdd; 

         elseif(strcmp(obj.control_type{ind_subchain,ind_task},'tracking'))
             
             if(strcmp(obj.type_of_traj{ind_subchain,ind_task},'func')) 

                p=feval(obj.trajectories{ind_subchain,ind_task}.p,t);
                pd=feval(obj.trajectories{ind_subchain,ind_task}.pd,t);
                pdd=feval(obj.trajectories{ind_subchain,ind_task}.pdd,t);

             elseif(strcmp(obj.type_of_traj{ind_subchain,ind_task},'sampled'))

                % index to the current value
                [~,ind] = min(abs(obj.trajectories{ind_subchain,ind_task}.time-t));
                %DEBUG
                %cur_time = f(ind); % Finds first one only! 
                %disp(cur_time);
                %--
                p  = obj.trajectories{ind_subchain,ind_task}.p(:,ind);
                pd = obj.trajectories{ind_subchain,ind_task}.pd(:,ind);
                pdd= obj.trajectories{ind_subchain,ind_task}.pdd(:,ind);

             end
         end
      end      
      
   end
    
end