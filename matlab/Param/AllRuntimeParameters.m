
%%%;;

%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 20;
time_struct.step = 0.001;

%% for simulation 
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
% define the type of integration of the sytem of differential equation
fixed_step = false; %true;
torque_saturation =10000000000000; % high value no saturation

%% TASK PARAMETERS
%name_dat = 'sere/LBR4p5.0_scene5_UF_repellers_on_elbow__atrtactive_point_on_ee_fit5_SERE';
%name_dat = 'sere/LBR4p9.0_scene5_GHC_table_and_an_one_attractive_point_and_posture_task_SERE';
%name_dat = 'LBR4p8.0_scene9_GHC_test_wall_and_two_attractive_point';
%name_dat = 'LBR4p11.0_scene9_UF_mulitple_task_stability_Null_space_projectors';
%name_dat = 'LBR4p10.0_scene10_UF_lemniscate';
%name_dat = 'LBR4p12.0_scene0_UF_test_elastic_reference';
name_dat = 'LBR4p2.0_scene2_two_regulation_task_and_wall_obstacle';
path=LoadParameters(name_dat);
load(path);

%% SCENARIO
name_scenario = 'lbr_scenario2'; %lbr_scenario5.1,'lbr_scenario9','lbr_scenario10';

%% STARTING CONDITION FOR SIMULATION
% TODO generalize for multichain
qi{1} = qz;
%qi{1} = zeros(1,chains.GetNumLinks(1)); %stretched arm
qdi{1} = zeros(1,chains.GetNumLinks(1));
options= [];
simulator_type = {'rbt'};

%% Parameters Dependant on the type of controller

%%%EOF

switch CONTROLLERTYPE
    case 'UF'
        UF_RuntimeParameters
    case 'GHC'
        GHC_RuntimeParameters
    otherwise
        warning('Unexpected control method')
end


%% DO NOT CHANGE THIS PART!


% backup data 
rawTextFromStorage = fileread(which(mfilename));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');

% join the general static parameter with the particular static one
rawTextFromStorage = strcat(rawTextFromStorage,rawTextFromStoragePart);

