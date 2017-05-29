%configuration file for the icub sitting stand up simulation
%%%;;

%% change folder (move to the folder with the simulink scheme)
name_simulink_model = 'TB_StandUp';
fullPath = which('find_simulatorIcubSim.m');
path = fileparts(fullPath);
path = strcat(path,'/',name_simulink_model);
cd(path)
%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 4;
time_struct.step = 0.01;

%% TASK PARAMETERS
name_dat = 'iCub_stand_up_sim_1.0'; % this is the name to give to the folder where im going to save the results
%path=LoadParameters(name_dat);
%load(path);
%% TYPE OF CONTROLLER
CONTROLLERTYPE ='BalanceController';   % GHC or UF 
%%

%SUBCHAIN PARAMETERS
subchain1 =  {'com'};
target_link{1} = subchain1;


%% Robot
bot1 = iCub('icubGazeboSim');
chain_1 = DummyRvc_iCub(bot1,'l_sole');
robots{1} = chain_1;
chains = SubChains(target_link,robots,bot1);

%% SCENARIO
name_scenario = 'Icub_stand_up';

%% RBT SIMULATOR PARAMETERS
time_sym_struct = time_struct;
time_sym_struct.step = 0.01;
% TODO generalize for multichain

simulator_type = {'icub_matlab_sim'};
%% TODO for now im not gonna use the structure that i used for the matlab simulator. Im going to use only init_contact_state
params.init_contact_state = [1 1]; 
params.feet_on_ground = params.init_contact_state;         %either 0 or 1; [left,right] (in the simulator)
params.numContacts = sum(params.feet_on_ground,2);

params.tStart   = time_struct.ti;
params.tEnd     = time_struct.tf;
params.sim_step =  0.01;%time_struct.step;
   


    %% parameters for controller and fitness (fitnessHumanoidsIcubStandUp)
    %xComfinal = [-0.120249695321353,-0.0680999719842103,0.369603821651986]';
    % standing_pose: -10   0  0, -20  30  0  45  0, -20  30  0  45  0, 25.5   0   0  -18.5  -5.5  0, 25.5   0   0  -18.5  -5.5  0
    % sitting_pose: 10   0  0, -20  30  0  45  0, -20  30  0  45  0,  90    0   0  -90    -5.5  0,  90    0   0   -90   -5.5  0
    params.qfinalSitting = [-10   0  0, -20  30  0  45  , -20  30  0  45  , 25.5   0   0  -18.5  -5.5  0,25.5   0   0  -18.5  -5.5  0]'*(pi/180);  
    params.qfinal        = [-10   0  0, -20  30  0  45  , -20  30  0  45  , 25.5   0   0  -18.5  -5.5  0,25.5   0   0  -18.5  -5.5  0]'*(pi/180);   
    params.tswitch       = 1;



%%  REFERENCE PARAMETERS

bot1.SetWorldFrameiCub(params.qjInit,params.dqjInit,params.dx_bInit,params.omega_bInit,params.root_reference_link);

deg = pi/180;
% primary trajectory
traj_type = {'cartesian'};
control_type = {'x'};
type_of_traj = {'func'};
geometric_path = {'AdHocBalance'};
time_law = {'none'};
%parameters first chains
geom_parameters{1,1} =  [5, 5 ,     2 ,...
                        bot1.init_state.xCoMRef(1),bot1.init_state.xCoMRef(2),bot1.init_state.xCoMRef(3),...
                        0.0167667444901888,-0.0681008604452745,0.503988037442802];  
dim_of_task{1,1}=[1;1;1]; 

% secondary trajectory (Not used)
traj_type_sec = {'none'};
control_type_sec = {'rpy'};
type_of_traj_sec = {'func'};
geometric_path_sec = {'fixed'};
time_law_sec = {'linear'};
%parameters first chains
geom_parameters_sec{1,1} = [pi/2 0 -pi/2]; % regulation
dim_of_task_sec{1,1}={[1;1;1]};


%% Parameters Dependant on the type of controller

%%%EOF

switch CONTROLLERTYPE
    case 'BalanceController'
        disp('UF_RUNTIMEPARAM')
        
        %%%;;
        
        %% PRIMARY REFERENCE PARAMETERS (this parameter only works if one of the specific trajectory has runtime parameters)
        numeric_reference_parameter{1,1}=[0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]'; % is not used but just to be compliant with the input structure
        secondary_numeric_reference_parameter{1,1} = []; % not used
        %% ALPHA PARAMETER (not used)
        %constant alpha
        choose_alpha = 'constant';  % RBF , constant, handTuned
        value1 = 0*ones(chains.GetNumTasks(1));
        values{1} = value1;
        value_range_for_optimization_routine = [-0.5 , 1.5]; % this is a trick that im using to provide bound to the optimization procedure for parametric reference
        %% CONTROLLER PARAMETER
        combine_rule = {'sum'}; % sum or projector (with sum reppelers are removed)
        %% CONSTRAINTS PARAMETERS
        constraints_values = bot1.createConstraintsVector;
        for k = 1:2:length(constraints_values)
            constraints_functions{k} = 'LinInequality';
            constraints_functions{k+1} = 'LinInequality2';
        end
        %% with the empty constraints it means that i compute the constraints directly inside the fitness function and i provide the result through the input of the empty constraints
        constraints_functions{end+1} = 'EmptyConstraints'; 
        constraints_values = [constraints_values,nan];   % vector that contains some constant that are used by the function in constraints_functions to compute the constraints_violation
        constraints_type = ones(1,length(constraints_values)); % vector that specifies if the constraints is a equality or an inequality. 1 disequality 0 equality
        activate_constraints_handling = false;
        %% INSTANCE PARAMETER
        preprocessing = @EmptyPreprocessing;
        run_function = @RobotExperiment;
        fitness = @fitnessHumanoidsIcubStandUpSearchFreeSolutionSimulink;
        clean_function = @RobotExperimentCleanData;
        
        input{1} = simulator_type{1};  % rbt / v-rep
        input{2} = params;
        input{3} = time_sym_struct;
        input{4} = [];                 % here i have to insert the controller i will do that in init()
       
        
        %% CMAES PARAMETER
        %--- Starting value of parameters
        generation_of_starting_point = 'test'; % 'test':user defined by user_defined_start_action 'given':is redundant with test  'random': random starting point
        %init_parameters = 6;
       
        user_defined_start_action =  [0.1,0.2,1.11440342040965,1.60499168353230,1.46748425429034,...
                                     -0.02800863753444,-0.0278361490435566,-0.0146154272285895,0.0134390845173483,-0.00801177055714880,...
                                     0.350954589796539,0.364988639468307,0.365816381480233,0.389461591950187,0.402856738959637]; 
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        niter = 100;  %number of generations
        %cmaes_value_range = [-14 , 14];  % boudn that define the search space
        cmaes_value_range{1} = [ 0, 0, 0, 0, 0, -0.12,-0.12,-0.12,-0.12,-0.12,  0.36,0.36,0.36,0.36,0.36 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 2, 2, 2, 2, 2,  0.016,0.016,0.016,0.016,0.016, 0.50,0.50,0.50,0.50,0.50];  % upper bound that define the search space
        learn_approach = '(1+1)CMAES'; %CMAES (1+1)CMAES
        %--- Parameter for constraints method
        method_to_use = 'nopenalty';  % adaptive , vanilla , empty
        epsilon = 0.001*ones(1,length(constraints_functions)); %vector with a number of value related to the number of constraints (used only with Aaptive constraints)
        %% FITNESS PARAMETERS
        
        %%%EOF
    otherwise
        warning('Unexpected control method')
end

%% DO NOT CHANGE THIS PART!

% backup data
rawTextFromStorage = fileread(which(mfilename));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');