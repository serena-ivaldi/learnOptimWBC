close all
clear variables
clc

%% only for debug of the remote api library
% if libisloaded('remoteApi')
%    unloadlibrary remoteApi
% end
%% 
check_scene = false;
% GENERAL PARAM 

time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.001;

%SUBCHAIN PARAMETERS 
subchain1 = [7];
target_link{1} = subchain1;

% matlab model
[LBR4p] = MdlLBR4p();

% this is usefull to check the geometric correspondance
%between vrep and matlab
%LBR4p.teach();

% vrep model
v = VAREP('~');%,'nosyncronous');
v_arm = VAREP_arm(v, 'LBR4p','fmt','%s_joint%d');

% set the first joint position 
v_arm.setq(qr)

% this is usefull to check the geometric correspondance
%between vrep and matlab
%v_arm.teach(); 


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
geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3]; % Circular trajectory 
dim_of_task{1,1}={[1;1;1]};


% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();



%% check scene
if check_scene
   
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
else
   % i use the same integration step of vrep
   if(v.syncronous)
      time_struct.step = v.GetSimDelta();
   end

   % i define the orientation for the inverse kinematic 
   fixed_orientation = roty(90);
   T = eye(4);
   T(1:3,1:3) = fixed_orientation;


   % start simulation
   v.simstart();
   q = qr';
   qd = zeros(7,1);
   q_meas = q;
   lambda = 0.001;
   % open loop control (only joints reference)
   for i = time_struct.ti:time_struct.step:time_struct.tf

      t = v.GetSimTime();
      [p,pd,pdd]=reference.GetTraj(1,1,t);
      T(1:3,4) = p;
      
      % inversione cinematica
      %q = LBR4p.ikine(T,'pinv');
      
      % inversion with jacobian
      J = LBR4p.jacob0(q,'trans');
      damped = J'/(J*J' - eye(3)*lambda);
      qd_new = damped*pd;
      
      q_new = (qd_new - qd)/time_struct.step;
      q = q + q_new
    
       v_arm.SetTargetQ(q);
      %v_arm.SetTargetQd(qd);
      
      if(v.syncronous)
         v.SendTriggerSync()
      end
     
      % update q and qd
      q_meas = v_arm.getq();
      qd_meas= v_arm.GetQd();
      %q = q_meas'; 
      qd = qd_new;
      
   end


   v.simstop();


end




