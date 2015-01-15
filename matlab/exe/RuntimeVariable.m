
%%%;;

%GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

% for simulation 
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
% define the type of integration of the sytem of differential equation
fixed_step = false;

% TASK PARAMETERS
name_dat = 'LBR4p5.1__scene5_test_controller';
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
%numeric_theta = [8.780098 8.765290 8.461586 11.204990 11.173776 8.551731 8.427082 9.581020 9.647632 7.245557 2.586975 9.250070 6.977037 7.210969 6.895398 6.914006 0.522321 8.655853 5.843199 6.691017 3.625846 5.457990 0.992693 4.097152 5.452603 2.960787 6.540737 4.252656 4.416881 10.533105 5.418901 1.855286 3.491727 1.727777 2.360968 1.729813 2.491500 1.261090 2.224534 6.177421 ];  
numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;


%CONTROLLER PARAMETERS
max_time = 100; %50
combine_rule = {'projector'}; % sum or projector
% with this term i introduce a damped least square structure inside my
% controller if regularizer is 0 i remove the regularizer action 
regularizer_chain_1 = [0 1]; %0.1
regularized_chain_2 = [1];
regularizer{1} = regularizer_chain_1;
regularizer{2} = regularized_chain_2;

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