
%%%;;

%GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

% for simulation 
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
fixed_step = false;

% TASK PARAMETERS
name_dat = 'LBR4p5.0__scene5_repellers_on_elbow__atrtactive_point_on_ee_fit5';
path=LoadParameters(name_dat);
load(path);

%SCENARIO
name_scenario = 'lbr_scenario5';

% REPELLERS PARAMETERS
rep_obstacle_ref = [1];

%ALPHA PARAMETERS
%rbf
number_of_basis = 10;
redundancy = 3;
range = [0 , 12];
precomp_sample = false;
% value of theta that we have to change when we want to execute the result
% from the optimization step
numeric_theta = 2*ones(number_of_basis,1);  
%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;


%CONTROLLER PARAMETERS
max_time = 50;
combine_rule = {'sum'}; 

% INSTANCE PARAMETERS and STARTING CONDITIONS FOR INTEGRATION (only qi and qdi)
qi{1} = qz;
qdi{1} = zeros(1,chains.GetNumLinks(1));
options= [];
simulator_type = {'rbt'};

% CMAES PARAMETER
% starting value of parameters
init_parameters = 6;
explorationRate =0.1;%[0, 1]
niter = 80;

%%%EOF