close all
clear variables
clc

%% only for debug of the remote api library
% if libisloaded('remoteApi')
%    unloadlibrary remoteApi
% end
%% 
op_selection = 'control';
control = 'fb_tracking_vel_joint';
% GENERAL PARAM 

time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.001;

%SUBCHAIN PARAMETERS 
subchain1 = [7];
target_link{1} = subchain1;

% matlab model
[LBR4p] = MdlLBR4p();

% vrep model
v = VAREP('~');%,'nosyncronous');
v_arm = VAREP_arm(v, 'LBR4p','fmt','%s_joint%d');

%desired_pose pointer
des_pos = v.object('cur_pos');

% set the first joint position 
v_arm.setq(qr)

% this is usefull to check the geometric correspondance
%between vrep and matlab



% test of the dynamic control cycle (position and velocity)
% activate servo controllers on the vrep robot!

% generate 
% REFERENCE PARAMETERS
traj_type = {'cartesian_x'};
control_type = {'tracking'};
type_of_traj = {'func'};
geometric_path = {'circular'};
time_law = {'linear'};
%parameters first chains
geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 0.5 0.3]; % Circular trajectory 
dim_of_task{1,1}={[1;1;1]};


% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();


switch op_selection
   
   case 'teach'
      % this is usefull to check the geometric correspondance
      %between vrep and matlab
      LBR4p.teach();
      v_arm.teach(); 
   case 'check_scene'
   
      time_struct_draw.ti = 0;
      time_struct_draw.tf = 10;
      time_struct_draw.step = 0.1;
      name_traj = 'circle';

      allpath=which('find_traj.m');
      path=fileparts(allpath);
      path = strcat(path,'/',name_traj,'.csv');
      fileID = fopen(path,'w');

      p_disp = [];
      for i = time_struct_draw.ti:time_struct_draw.step:time_struct_draw.tf

         [p,pd,pdd]=reference.GetTraj(1,1,i);
         p_disp=[p_disp;p'];
         fprintf(fileID,'%d,%d,%d\n',p(1),p(2),p(3));
      end
      fclose(fileID);
      hold on;
      LBR4p.teach(); 
      plot3(p_disp(:,1),p_disp(:,2),p_disp(:,3));
   case 'control'
      % i use the same integration step of vrep
      if(v.syncronous)
         time_struct.step = v.GetSimDelta();
      end
      
      switch control
      
         case 'ff_regulation'
            %% open loop regulation
            q_goal = [pi/2,pi/2,0,-pi/2,0,0,0];
            v_arm.SetTargetQ(q_goal);
            
            v.simstart();
            Kp = 50; 
            for i = time_struct.ti:time_struct.step:time_struct.tf
               
               q_cur = v_arm.getq();
               q_e = Kp*(q_goal - q_cur);
               
               
               if(v.syncronous)
                  v.SendTriggerSync()
               end
               
            end
            
             v.simstop();

         case 'ff_tracking_pos'
            %%   open loop tracking (only joints reference)  inverse kinematics 
            % i define the orientation for the inverse kinematic 
            fixed_orientation = roty(90);
            T = eye(4);
            T(1:3,1:3) = fixed_orientation;
            
           
            % start simulation
            v.simstart();
            q = qr';
           
            for i = time_struct.ti:time_struct.step:time_struct.tf
      
               t = v.GetSimTime();
               [p,pd,pdd]=reference.GetTraj(1,1,t);
               des_pos.setpos(p');
               T(1:3,4) = p;

               % inversione cinematica
               q = LBR4p.ikunc(T);

               v_arm.SetTargetQ(q);
               

               if(v.syncronous)
                  v.SendTriggerSync()
               end

               
            
            end
      
      
         v.simstop();
         
         case 'fb_tracking_vel_joint' % in the vrep model yuo have to remove the low level control loop (pid or spring-damper)
            
             v.simstart();
             q_meas= qr';
             qd_cur = zeros(7,1);
             lambda = 0.1;
             Kp = 20;
             Ki = 100;
             % intial position attached to the trajectory
             [p,pd,pdd]=reference.GetTraj(1,1,0);
             fixed_orientation = roty(90);
             T = eye(4);
             T(1:3,1:3) = fixed_orientation;
             T(1:3,4) = p;
             q_des = LBR4p.ikunc(T)';
             e_int = 0;
             for i = time_struct.ti:time_struct.step:time_struct.tf
                
                % reference in cartesian
                t = v.GetSimTime();
                [p,pd,pdd]=reference.GetTraj(1,1,t);
                des_pos.setpos(p');
                
                % inversion with jacobian
                J = LBR4p.jacob0(q_des,'trans');
                J_damp = J'/(J*J' + eye(3)*lambda);
                qd_des = J_damp*pd;
                
                % euler integration of velocity(ode1)
                q_des = q_des + qd_des*time_struct.step;
                
                
                e_int = e_int + (q_des - q_meas)*time_struct.step;
                control = Kp*(q_des - q_meas) + Ki*(e_int) + qd_des;
                
                v_arm.SetTargetQd(control);
                
                if(v.syncronous)
                  v.SendTriggerSync()
               end

                % update q and qd
               q_meas = v_arm.getq()';   
             end
            
            v.simstop();
            
      end

end




