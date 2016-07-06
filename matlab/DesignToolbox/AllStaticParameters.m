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
subchain1 =  {'r_gripper','r_elbow_1','none'};
target_link{1} = subchain1;


%% Robot
bot1 = iCub('model_arms_torso_free');
chain_1 = DummyRvc_iCub(bot1,'l_sole'); 
robots{1} = chain_1;
chains = SubChains(target_link,robots,bot1);
%%  REFERENCE PARAMETERS
deg = pi/180;
% primary trajectory
% traj_type = {'impedance'};
% control_type = {'x'};
% type_of_traj = {'func'};
% geometric_path = {'fixed'};
% time_law = {'none'};
% %parameters first chains
% geom_parameters{1,1} = [0.30 -0.71 0.5]; 
% %geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
% dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.n,1); %dim_of_task{1,4}=ones(bot1.n,1);

traj_type = {'cartesian','cartesian','joint'};
control_type = {'x','x','none'};
type_of_traj = {'sampled','sampled','sampled'};
geometric_path = {'fixed','fixed','fixed'};
time_law = {'none','none','none'};
%parameters first chains
geom_parameters{1,1} = [0.45,-0.1,0.7]; 
geom_parameters{1,2} = [0.25,-0.25,0.7];
geom_parameters{1,3} = [0;0.785398163397448;0;0;-0.349065850398866;0.523598775598299;0;0;0.785398163397448;0;0;0;0.523598775598299;0;0;0;0]';
dim_of_task{1,1}=[1;1;1]; dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.ndof,1); 

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
traj_type_sec = {'none','none','none'};
control_type_sec = {'rpy','rpy','rpy'};
type_of_traj_sec = {'func','func','func'};
geometric_path_sec = {'fixed','fixed','fixed'};
time_law_sec = {'linear','linear','linear'};
%parameters first chains
geom_parameters_sec{1,1} = [pi/2 0 -pi/2]; % regulation
geom_parameters_sec{1,2} = [-0.309 -0.469 0.581];
geom_parameters_sec{1,3} = [-0.309 -0.469 0.581];
geom_parameters_sec{1,4} = [-0.309 -0.469 0.581];
geom_parameters_sec{1,5} = [-0.309 -0.469 0.581];
geom_parameters_sec{1,6} = [-0.309 -0.469 0.581];
geom_parameters_sec{1,7} = [120 116 90 0 0 0 0]* deg; 
dim_of_task_sec{1,1}=[1;1;1]; 
dim_of_task_sec{1,2}=[1;1;1]; 
dim_of_task_sec{1,3}=[1;1;1]; 
dim_of_task_sec{1,4}=[1;1;1]; 
dim_of_task_sec{1,5}=[1;1;1]; 
dim_of_task_sec{1,6}=[1;1;1]; 
dim_of_task_sec{1,7}= ones(bot1.ndof,1); 

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
id = 'iCub';
name_backup = strcat(id,'.m');
%namebot_scene#_briefdescription.mat
number = '1.0';
name_file = strcat(id,'_',number,'.mat');

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

existence = exist(strcat(path,'/back_data/',name_file),'file');
if(~existence)
    fileID = fopen(strcat(path,'/back_data/',name_file),'w');
    fprintf(fileID,'%s',rawTextFromStorage);
    fclose(fileID);
    disp('FINISH!')
else
    adv = strcat('The file: /',name_file,' allready exist');
    b=questdlg(adv, 'Overwrite?','Yes','No','No');
    switch b
        case 'Yes'
            fileID = fopen(strcat(path,'/back_data/',name_file),'w');
            fprintf(fileID,'%s',rawTextFromStorage);
            fclose(fileID);
            disp('FINISH!')
    end
end


