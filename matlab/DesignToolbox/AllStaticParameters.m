clear variables
close all
clc

%%%;;

%% comment
%this file describe a test task for the elastic reference
%
%
%% TYPE OF CONTROLLER 
CONTROLLERTYPE ='UF';   % GHC or UF
%%

%SUBCHAIN PARAMETERS 
subchain1 = [7];
target_link{1} = subchain1;

%% Robot
[bot1] =  MdlLBR4pSimple();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%  REFERENCE PARAMETERS
deg = pi/180;
% primary trajectory
traj_type = {'impedance'};
control_type = {'x'};
type_of_traj = {'func'};
geometric_path = {'fixed'};
time_law = {'none'};
%parameters first chains
geom_parameters{1,1} = [0.30 -0.71 0.5]; 
%geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.n,1); %dim_of_task{1,4}=ones(bot1.n,1);

% traj_type = {'cartesian'};
% control_type = {'rpy'};
% type_of_traj = {'func'};
% geometric_path = {'fixed'};
% time_law = {'none'};
% %parameters first chains
% geom_parameters{1,1} = [pi/2 0 -pi/2]; 
% %geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
% dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.n,1); %dim_of_task{1,4}=ones(bot1.n,1);

% secondary trajectory
traj_type_sec = {'none'};
control_type_sec = {'rpy'};
type_of_traj_sec = {'func'};
geometric_path_sec = {'fixed'};
time_law_sec = {'linear'};
%parameters first chains
geom_parameters_sec{1,1} = [pi/2 0 -pi/2]; % regulation
dim_of_task_sec{1,1}={[1;1;1]};

% traj_type_sec = {'cartesian_x'};
% control_type_sec = {'regulation'};
% type_of_traj_sec = {'none'};
% geometric_path_sec = {'none'};
% time_law_sec = {'linear'};
% parameters first chains
% geom_parameters_sec{1,1} = [0.30 -0.71 0.5]; % regulation
% dim_of_task_sec{1,1}={[1;1;1]};


%% FROM THIS POINT YOU CAN FIND PARAMETERS IN THE STATIC PARAMETERS FILE RELATED TO EACH ALGORITHM 

%%%EOF

switch CONTROLLERTYPE
    case 'UF'
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
        UF_StaticParameters
    case 'GHC'
        GHC_StaticParameters
    otherwise
        warning('Unexpected control method')
end


%% Name of the file (backup and .mat)
% id specify wich is the backup data that I have to look at. 
% i have to set the name of the robot plus a number equal to the number of experiment for that scenario 
% like bot#.# (where n.i means that the file is reffered to the n-scenario and is the i-th data setting)
% multiple data setting for the same scenario 
id = 'lwrsimple1.0';
name_backup = strcat(id,'.m');
%namebot_scene#_briefdescription.mat
name_file = 'scene_test_obs';
name_file = strcat(id,'_',name_file,'.mat');

%% DO NOT CHANGE THIS PART!
% build the correct path for .mat
allpath=which('FindData.m');
path=fileparts(allpath);
save(strcat(path,'/datamat/',name_file));

% backup data 
rawTextFromStorage = fileread(which(mfilename));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');

% join the general static parameter with the particular static one
rawTextFromStorage = strcat(rawTextFromStorage,rawTextFromStoragePart);

existence = exist(strcat(path,'/back_data/',name_backup),'file');
if(~existence)
    fileID = fopen(strcat(path,'/back_data/',name_backup),'w');
    fprintf(fileID,'%s',rawTextFromStorage);
    fclose(fileID);
    disp('FINISH!')
else
    adv = strcat('The file: /',name_backup,' allready exist');
    b=questdlg(adv, 'Overwrite?','Yes','No','No');
    switch b
        case 'Yes'
            fileID = fopen(strcat(path,'/back_data/',name_backup),'w');
            fprintf(fileID,'%s',rawTextFromStorage);
            fclose(fileID);
            disp('FINISH!')
    end
end


