
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
fixed_step = false;

%% TASK PARAMETERS
name_dat = 'LBR4p8.0__scene9_GHC_test_wall_and_two_attractive_point';
%name_dat = 'LBR4p5.0__scene5_repellers_on_elbow__atrtactive_point_on_ee_fit5';
path=LoadParameters(name_dat);
load(path);

%% SCENARIO
name_scenario = 'lbr_scenario9';

%% STARTING CONDITION FOR SIMULATION
% TODO generalize for multichain
qi{1} = qz;
qdi{1} = zeros(1,chains.GetNumLinks(1));
options= [];
simulator_type = {'rbt'};

%% Parameters Dependant on the type of controller
switch CONTROLLERTYPE
    case 'UF'
        UF_RuntimeParameters
    case 'GHC'
        GHC_RuntimeParameters
    otherwise
        warning('Unexpected control method')
end

%%%EOF