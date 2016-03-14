classdef  References < handle
    
   properties
      target_link           %  vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
      type;                 % cartesian_x,cartesian_rpy, joint vector or empty 
      control_type;         % tracking,regulation vector
      traj;                 % circular, rectilinear, point-point_quintic, point-point_trapezoidal vector
      geom_parameters;      % vector of parameters that define the properties of every trajectories (both functional and sampled)
      time_law;             % exponential,linear,constant,trapezoidal
      %time_parameters;    % vector of the time parameters of the specified time law
      time_struct           % struct with time_struct.ti time_struct.tf time_struct.step
      mask;                 % vector of vector(3 or DOF) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      type_of_traj;         % sampled func
      parameter_dim;        % lenght of the vector of parameter that i use for the reference (cell array)
      cur_param_set;        % this is a cell array of values that is used when we use a parametric trajectory so we can optimize this parameters.
      trajectories;         % cell array with the sampling of the trajectory with position velocity and desired acceleration  
                            % in case of sampled trajectory i build up a struct with four fields "p" "pd" "pdd" (row matrix) contains the value of the  trajectory 
                            %and "time" (row vector) that contains the sampling time
                    
   end
       
    
   methods
      %#TODO add control of the input
      function obj = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,mask,type_of_traj,varargin) % i can specify through varargin the time duration of the sampled trajectories 
          
         obj.target_link = target_link;
       
         %warning: getnameidx is in the financial toolbox
         %if(getnameidx({'joint' 'cartesian' 'impedance'} , type) ~= 0 )
            obj.type = type;
         %end
         %if(getnameidx({'x' 'rpy'} , control_type) ~= 0)
            obj.control_type = control_type;
         %end
         %if(getnameidx({ 'circular','rectilinear', 'point-point_quintic' , 'point-point_trapezoidal' 'none'} , traj) ~= 0)
            obj.traj = traj;
         %end
         
         %if(getnameidx({'exponential','linear','constant','trapezoidal' 'none'} , time_law) ~= 0)
            obj.time_law = time_law;
         %end
         
         obj.geom_parameters = geom_parameters;
         obj.time_struct = time_struct;
         obj.mask = mask;
         
         %if(getnameidx({ 'sampled', 'func'} , type_of_traj) ~= 0)
            obj.type_of_traj = type_of_traj;
         %end
         
         
         
            
      end  
      
      function BuildTrajs(obj)
         
          for i = 1:size(obj.target_link,1)
             for j = 1:size(obj.target_link{i},2)
               % initialize the structure for each task for each kinematic chain to manage parameters in the reference   
               obj.parameter_dim{i,j} = 0;
               obj.cur_param_set{i,j} = [];
               obj.SetTraj(i,j)
             end
          end    
      end
      
      function n = GetDimTask(obj,ind_subchain,ind_task)
          n = nnz(obj.mask{ind_subchain,ind_task});
      end
      
      function [p,pd,pdd]=GetTraj(obj,ind_subchain,ind_task,t)   
         if(strcmp(obj.type_of_traj{ind_subchain,ind_task},'func')) 
            if(obj.parameter_dim{ind_subchain,ind_task} > 0 ) % i need this block for elastic trajectory because i need to change the trajectory each generation
               p=feval(obj.trajectories{ind_subchain,ind_task}.p,t,obj.cur_param_set{ind_subchain,ind_task});
               p=p(1:3,:); % i have to do that for manage the case in which i have constant function
               pd=feval(obj.trajectories{ind_subchain,ind_task}.pd,t,obj.cur_param_set{ind_subchain,ind_task});
               pd = pd(1:3,:); % i have to do that for manage the case in which i have constant function
               pdd=feval(obj.trajectories{ind_subchain,ind_task}.pdd,t,obj.cur_param_set{ind_subchain,ind_task});
               pdd = pdd(1:3,:); % i have to do that for manage the case in which i have constant function
            else
               p=feval(obj.trajectories{ind_subchain,ind_task}.p,t);
               p=p(1:3,:); % i have to do that for manage the case in which i have constant function
               pd=feval(obj.trajectories{ind_subchain,ind_task}.pd,t);
               pd = pd(1:3,:); % i have to do that for manage the case in which i have constant function
               pdd=feval(obj.trajectories{ind_subchain,ind_task}.pdd,t);
               pdd = pdd(1:3,:); % i have to do that for manage the case in which i have constant function
            end
         elseif(strcmp(obj.type_of_traj{ind_subchain,ind_task},'sampled'))
            % index to the current value
            [~,ind] = min(abs(obj.trajectories{ind_subchain,ind_task}.time-t));
            %DEBUG
            %cur_time = f(ind); % Finds first one only! 
            %disp(cur_time);
            %--
            p  = obj.trajectories{ind_subchain,ind_task}.p(1:3,ind);   % i have to do that for manage the case in which i have constant function
            pd = obj.trajectories{ind_subchain,ind_task}.pd(1:3,ind);  % i have to do that for manage the case in which i have constant function
            pdd= obj.trajectories{ind_subchain,ind_task}.pdd(1:3,ind); % i have to do that for manage the case in which i have constant function
         end
      end 
      
      function n=GetNumParam(obj,i)
         n =0;
         for j=1:size(obj.parameter_dim,2)  
           n_param = obj.parameter_dim{i,j};
           n = n + n_param;
         end
      end
      
      
   end
    
end