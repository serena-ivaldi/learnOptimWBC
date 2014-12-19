clear all
close all
clc

%%%;;





%% comment
%this file describe a trajecotry task for the e-e on a circle and 1
%repellers on the trajecotry
%
%
%%

%SUBCHAIN PARAMETERS 
subchain1 = [7];
target_link{1} = subchain1;

%% Robot
[bot1] = MdlLBR4p();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
type = {'cartesian_x'};
control_type = {'tracking'};
type_of_traj = {'func'};
traj = {'circular'};
time_law = {'linear'};
%parameters first chains
geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3]; % Circular trajectory 

% REPELLER PARAMETERS
% scenario dependant
rep_subchain = [7];
rep_target_link{1} = rep_subchain;
rep_type = {'cartesian_x'};
rep_mask {1,1}=[1,1,1];
rep_type_of_J_rep = {'DirectionCartesian'};
for ii=1:chains.GetNumChains()
    chain_dof(ii) = chains.GetNumLinks(ii);
end

%CONTROLLER PARAMETERS
metric = {'M'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
dim_of_task{1,1}=[1;1;1];

kp = [1700]; % row vector one for each chain
for i= 1:chains.GetNumChains()
   K_p = zeros(3,3,size(kp,2));
   K_d = zeros(3,3,size(kp,2));
   for par = 1:chains.GetNumTasks(i)
       K_p(:,:,par) = kp(i,par)*eye(3);  
       kd = 2*sqrt(kp(i,par));
       K_d(:,:,par) = kd*eye(3); 
   end
   Kp{i} = K_p;
   Kd{i} = K_d;
end


% INSTANCE PARAMETERS
fitness= @fitness4;





%%%EOF

%% Name of the file (backup and .mat)
% id specify wich is the backup data that I have to look at. 
% i have to set the name of the robot plus a number equal to the number of experiment for that scenario 
% like bot#.# (where n.i means that the file is reffered to the n-scenario and is the i-th data setting)
% multiple data setting for the same scenario 
id = 'LBR4p3.0';
name_backup = strcat(id,'.m');
%namebot_scene#_briefdescription.mat
name_file = '_scene3_ee_tracking_circ_obstacle_on_traj_1repellers_fit4';
name_file = strcat(id,'_',name_file,'.mat');

%% DO NOT CHANGE THIS PART!
% build the correct path for .mat
allpath=which('FindData.m');
path=fileparts(allpath);
save(strcat(path,'/datamat/',name_file));

% backup data 
rawTextFromStorage = fileread(which(mfilename));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');
fileID = fopen(strcat(path,'/back_data/',name_backup),'w');
fprintf(fileID,'%s',rawTextFromStorage);
fclose(fileID);
disp('FINISH!')

