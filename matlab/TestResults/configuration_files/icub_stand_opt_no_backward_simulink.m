%configuration file for the icub sitting stand up simulation
%%%;;
%% prepare the system for the simulink executable (every function that use simulink has to use SimulinkInitializationExperiment(...) !)

% params is shipped inside the simulink-thread through the 'input' cell
% vector. input variables has renamed as 'input_for_run' inside the instance object
name_simulink_folder  = 'TB_StandUp';
name_simulink_schemes = 'torqueBalancing2012b';
scenario_name         = 'sit_icub_to_optimize_0_1.world';
% just temporary until codyco is updated on every machine
codyco                = 'old'; % old or new depending on your codyco installation (2017 codyco version = old 2018 codyco version = new)
% here i build the class that is responsible of the communication among
% matlab processes
messenger             = Messaging.TB_StandUpMessage();

%% not change this part!
params.name_simulink_schemes = name_simulink_schemes;
params.codyco                = codyco;
params.messenger             = messenger;
params.scenario_name         = scenario_name;
[params.simulink_schemes_global,params.path_to_local_simscheme] = SimulinkInitializationExperiment(name_simulink_folder,scenario_name,codyco);
%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 4.5;
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
bot1 = iCub('icubGazeboSimSimulink');
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
params.qjInit      = bot1.InitializeStateicubGazeboSim(params.feet_on_ground);
% i need to add +2 to the joitn velocity values because of the difference
% joints state between the matlba icub and gazebo icub (look at icub class for more information) 
params.dqjInit     = zeros(bot1.ndof + 2 ,1);
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
% feet size
params.footSize  = [-0.07  0.12 ;    % xMin, xMax
                    -0.045 0.05];    % foot_xlength, foot_ylength 
params.foot.xmin    = params.footSize(1,1);
params.foot.xmax    = params.footSize(1,2);
params.foot.ymin    = params.footSize(2,1);
params.foot.ymax    = params.footSize(2,2);

params.footSizeForOpitmization = [-0.07  0.12 ;    % xMin, xMax
                                  -0.045 0.05];      % yMin, yMax   


    %% parameters for controller and fitness (fitnessHumanoidsIcubStandUp)
    %xComfinal = [-0.120249695321353,-0.0680999719842103,0.369603821651986]';
    % standing_pose: -10   0  0, -20  30  0  45  0, -20  30  0  45  0, 25.5   0   0  -18.5  -5.5  0, 25.5   0   0  -18.5  -5.5  0
    % sitting_pose: 10   0  0, -20  30  0  45  0, -20  30  0  45  0,  90    0   0  -90    -5.5  0,  90    0   0   -90   -5.5  0
    params.qfinalSitting = [60.0000001093867 0.618804962651733 0.399999267875981... 70
                            -67.2000001361580 34.0999961120969 4.79796140019389 43.1917772138638...
                            -67.2000001361580 34.0999961120969 4.79796140019389 43.1917772138638...
                            84.2999649174303	0.761524617074400	0.0867967193079845	-99.2529302018096	-15.8102389048266	0.0632937999425440...
                            84.2999649174303	0.761524617074400	0.0867967193079845	-99.2529302018096	-15.8102389048266	0.0632937999425440]'*(pi/180);  
    params.qfinal        = [-10   0  0, -20  30  0  45  , -20  30  0  45  , 25.5   0   0  -18.5  -5.5  0,25.5   0   0  -18.5  -5.5  0]'*(pi/180);   
    params.tswitch       = 1.5;
    % if this parameter is true we fix the desired com value(the com value is specify inside the trajectory block in the simulink)
    % if this parameter if false we optinputimize the com trajectory even when the robt is sitting on the bench the bench
    params.fixedcombench = false;
    params.final_com     = [0.0167667444901888,-0.0681008604452745,0.503988037442802];

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
                        -0.0254627184564190	-0.0679301926936281	0.308024116757686,...
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
        disp('BALANCE_CONTROLLER_RUNTIMEPARAM')
        
        %%%;;
        
        %% PRIMARY REFERENCE PARAMETERS (this parameter only works if one of the specific trajectory has runtime parameters)
        % IMPORTANT!!!!! this value is used inside main exec to set the parameter that yuo want to test
        numeric_reference_parameter{1,1}=[0.385243701380465,0.812996966251098,0.725403083276668,0.919054324170722,1.44884212316731,-0.0375859449315661,-0.0141420455958836,0.00889677104501083,-0.0138436722679563,-0.0347847461961691,0.382693082842123,0.366609543042157,0.464354478552660,0.489918636980728,0.395451837082262]';
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
        torqueThreshold = 25;
        %% we do this small addition to limit the joint below a fixed treshold
        for iii = 47:92
            if(abs(constraints_values(iii))>torqueThreshold)
                if(constraints_values(iii)>0)
                    constraints_values(iii) = torqueThreshold;
                elseif(constraints_values(iii)<0)
                    constraints_values(iii) = -torqueThreshold;
                end      
            end
        end
        constraints_type = ones(1,length(constraints_values)); % vector that specifies if the constraints is a equality or an inequality. 1 disequality 0 equality
        %% to activate or disactive the constraints
        activate_constraints_handling = true;
        %% INSTANCE PARAMETER
        preprocessing = @EmptyPreprocessing;
        run_function = @RobotExperiment;
        fitness = @fitnessHumanoidsIcubStandUpOptSolutionNoBackwardSimulink;
        clean_function = @RobotExperimentCleanData;
        
        input{1} = simulator_type{1};  % rbt / v-rep
        input{2} = params;
        input{3} = time_sym_struct;
        input{4} = [];                 % here i have to insert the controller i will do that in init()
        
        
        %% CMAES PARAMETER
        %--- Starting value of parameters
        generation_of_starting_point = 'test'; % 'test':user defined by user_defined_start_action 'given':is redundant with test  'random': random starting point
        %init_parameters = 6;
      
        user_defined_start_action =   [0.334864347662051,0.769247868133844,0.574316421814835,0.951772057698620,1.47859968547875,...
                                      -0.0421739842002086,-0.0144987143004585,-0.00446705976414447,-0.0103957572854113,-0.0301487995300616,...
                                       0.393392500273063,0.373428273663188,0.463322910737024,0.481471759199476,0.395496648477922]; 
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        niter = 500;  %number of generations
        %cmaes_value_range = [-14 , 14];  % boudn that define the search space
        cmaes_value_range{1} = [ 0, 0, 0, 0, 0, -0.12,-0.12,-0.12,-0.12,-0.12,  0.36,0.36,0.36,0.36,0.36 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 2, 2, 2, 2, 2,  0.016,0.016,0.016,0.016,0.016, 0.50,0.50,0.50,0.50,0.50];  % upper bound that define the search space
        learn_approach = '(1+1)CMAES'; %CMAES (1+1)CMAES
        %--- Parameter for constraints method
        method_to_use = 'nopenalty';  % adaptive , vanilla , empty 'nopenalty'
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