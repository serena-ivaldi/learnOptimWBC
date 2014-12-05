close all
clear all
clc

%%%;;
%% comment
%this file describe two tasks a tracking task for the e-e on a circular
%trajecotry and a reglation task on the pose of the elbow (work with scenario1)
%
%%


%SUBCHAIN PARAMETERS 
subchain1 = [7 4];
target_link{1} = subchain1;


%% Robot
[LBR4p] = MdlLBR4p();
robots{1} = LBR4p;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
subchain1 = [7 4];
type = {'cartesian_x','cartesian_rpy'};
control_type = {'tracking','regulation'};
type_of_traj = {'func','func'};
traj = {'circular','none'};
time_law = {'linear','none'};

geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 -0.7 0.6]; % Circular trajectory
geom_parameters{1,2} = [0 0    pi/2]; % orientation regulation
%CONTROLLER PARAMETERS
metric = {'M','M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        

kp = [700, 700]; % row vector one for each chain
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
fitness= @fitness1;

%%%EOF

%% Name of the file (backup and .mat)
% id specify wich is the backup data that I have to look at. 
% i have to set the name of the robot plus a number equal to the number of scenario 
% like bot#.# (where n.i means that the file is reffered to the n-scenario and is the i-th data setting )
% multiple data setting for the same scenario 
id = 'LBR4p1';
name_backup = strcat(id,'.m');
%namebot_scene#_briefdescription.mat
name_file = 'LBR4p_scene1_wrist_ee_track_pose';
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

disp('FINISH!')

