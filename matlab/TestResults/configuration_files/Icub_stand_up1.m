%configuration file for the icub sitting stand up simulation
%%%;;

%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.01;

%% TASK PARAMETERS
name_dat = 'iCub_stand_up_1.0'; % this is the name to give to the folder where im going to save the results
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
%%  REFERENCE PARAMETERS
deg = pi/180;
% primary trajectory
traj_type = {'cartesian'};
control_type = {'x'};
type_of_traj = {'func'};
geometric_path = {'AdHocBalance'};
time_law = {'none'};
%parameters first chains
geom_parameters{1,1} =  [5 ,2 ,-0.120249695321353,-0.0680999719842103,0.369603821651986];
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


%% SCENARIO
name_scenario = 'Icub_stand_up';

%% RBT SIMULATOR PARAMETERS
time_sym_struct = time_struct;
time_sym_struct.step = 0.01;
% TODO generalize for multichain

simulator_type = {'icub_matlab'};
if strcmp(simulator_type{1},'rbt')
    % rbt sim
    qi{1} = qz;
    %qi{1} = zeros(1,chains.GetNumLinks(1)); %stretched arm
    qdi{1} = zeros(1,chains.GetNumLinks(1));
    options= [];
    % define the type of integration of the sytem of differential equation
    fixed_step = false; %true;
    torque_saturation =100000; % high value == no saturation
    maxtime = 100; % maximum time before a simulation is stopped for being too long
elseif strcmp(simulator_type{1},'icub_matlab') 
    %% here I build to different structure one for the controller and one for the simulator
    %% to manage contacts
    params.init_contact_state = [1 1 1 1]; 
    names         =  {'l_sole','r_sole','l_upper_leg','r_upper_leg'};   
    params.contact_sym = Contacts(params.init_contact_state,names);


    params.feet_on_ground = params.init_contact_state;         %either 0 or 1; [left,right] (in the simulator)
    params.numContacts = sum(params.feet_on_ground,2);
    if  params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1 && params.feet_on_ground(3) == 1  && params.feet_on_ground(4) == 1 
        % contact constraints for 2 feet on ground
        params.contactLinkNames      = {'l_sole','r_sole','l_upper_leg','r_upper_leg'};
    elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1 && params.feet_on_ground(3) == 1  && params.feet_on_ground(4) == 0
        % contact constraints for 2 feet on ground
        params.contactLinkNames      = {'l_sole','r_sole','l_upper_leg'};
    elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1 && params.feet_on_ground(3) == 0  && params.feet_on_ground(4) == 1
        % contact constraints for 2 feet on ground
        params.contactLinkNames      = {'l_sole','r_sole','r_upper_leg'};
    elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1 && params.feet_on_ground(3) == 0  && params.feet_on_ground(4) == 0
        % contact constraints for 2 feet on ground
        params.contactLinkNames      = {'l_sole','r_sole'};
    elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0 && params.feet_on_ground(3) == 0 && params.feet_on_ground(4) == 0
        % contact constraints for left foot on ground
        params.contactLinkNames      = {'l_sole'};
    elseif   params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1 && params.feet_on_ground(3) == 0 && params.feet_on_ground(4) == 0
        % contact constraints for right foot on ground
        params.contactLinkNames      = {'r_sole'};
    end
    % FLOATING BASE
    %params.active_floating_base = false;
    params.qjInit      = bot1.InitializeStateicubGazeboSim(params.feet_on_ground);
    params.dqjInit     = zeros(bot1.ndof,1);
    % icub starting velocity floating base
    params.dx_bInit    = zeros(3,1);
    params.omega_bInit = zeros(3,1);
    % root reference link;
    if params.feet_on_ground(1) == 1
        params.root_reference_link ='l_sole';
    else
        params.root_reference_link ='r_sole';
    end
    params.tStart   = time_struct.ti;
    params.tEnd     = time_struct.tf;
    params.sim_step =  0.01;%time_struct.step;
    params.demo_movements = 1;
    params.maxtime = 100;
    params.torque_saturation = 100000;
    params.integrateWithFixedStep = true;
    if params.integrateWithFixedStep   
        params.massCorr = 0.05;
    else
        params.massCorr = 0;
    end
    %% other parameters
    params.use_QPsolver = 0;                          %either 0 or 1
    params.pinv_tol           = 1e-8;
    params.pinv_damp          = 5e-6;
    params.reg_HessianQP      = 1e-3;
    % feet size
    params.footSize  = [0.07 0.03];    % foot_xlength, foot_ylength 
    %% parameters for controller and fitness (fitnessHumanoidsIcubStandUp)
    params.xComfinal = [0.0167667444901888;-0.0681008604452745;0.503988037442802];
    params.qfinal    = [-10   0  0, -20  30  0  45  0, -20  30  0  45  0, 25.5   0   0  -18.5  -5.5  0,25.5   0   0  -18.5  -5.5  0]'*(pi/180);   
end
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
        activate_constraints_handling = true;
        %% INSTANCE PARAMETER
        run_function = @RobotExperiment;
        fitness = @fitnessHumanoidsIcubStandUp;
        clean_function = @RobotExperimentCleanData;
        
        if strcmp(simulator_type{1},'rbt')
            % TODO generalize for multichain
            input{1} = simulator_type{1};  % rbt / v-rep
            input{2} = qi;                 % initial position
            input{3} = qdi;                % initial velocity
            %------
            input{4} = time_sym_struct;    %time struct for simulation with fixed step
            input{5} = [];                 % here i have to insert the controller i will do that in init()
            input{6} = fixed_step;         % if is true i use ode4 (runge-kutta)
            input{7} = torque_saturation;  % i define the torque saturation that i want to apply
            input{8} = maxtime;            % maxtime before the simulation is stopped because is too long
        elseif strcmp(simulator_type{1},'icub_matlab')
            input{1} = simulator_type{1};  % rbt / v-rep
            input{2} = params;
            input{3} = time_sym_struct;
            input{4} = [];                 % here i have to insert the controller i will do that in init()
            %input{5} = fixDistance;
        end
        %% CMAES PARAMETER
        %--- Starting value of parameters
        generation_of_starting_point = 'random'; % 'test':user defined by user_defined_start_action 'given':is redundant with test  'random': random starting point
        %init_parameters = 6;
        a = 5; b = 5; c = 5; d = 5; e = 6;
        user_defined_start_action = [a a a a a b b b b b c c c c c d d d d d e e e e e];
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        niter = 100;  %number of generations
        %cmaes_value_range = [-14 , 14];  % boudn that define the search space
        cmaes_value_range{1} = [-5,-5,-5,-5,-5, -0.12,-0.12,-0.12,-0.12,-0.12,  0.36,0.36,0.36,0.36,0.36 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 5, 5, 5, 5, 5,  0.016,0.016,0.016,0.016,0.016, 0.50,0.50,0.50,0.50,0.50];  % upper bound that define the search space
        learn_approach = 'CMAES'; %CMAES (1+1)CMAES
        %--- Parameter for constraints method
        method_to_use = 'vanilla';  % adaptive , vanilla , empty
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