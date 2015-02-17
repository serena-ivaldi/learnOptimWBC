
%%%;;

%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

%% for simulation 
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
% define the type of integration of the sytem of differential equation
fixed_step = true;

%% TASK PARAMETERS
%name_dat = 'sere/LBR4p5.0_scene5_UF_repellers_on_elbow__atrtactive_point_on_ee_fit5_SERE';
name_dat = 'LBR4p8.0_scene9_GHC_test_wall_and_two_attractive_point';
path=LoadParameters(name_dat);
load(path);

%% SCENARIO
name_scenario = 'lbr_scenario9';

%% STARTING CONDITION FOR SIMULATION
% TODO generalize for multichain
qi{1} = qz;
%qi{1} = zeros(1,chains.GetNumLinks(1));
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

