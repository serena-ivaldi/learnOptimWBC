%configuration file for the icub sitting stand up simulation
%%%;;

% here i open all the program that are necessary for the simulation
% execution and i create a symbolic link to matlab
% create symbolic link of matlab executable in TB_standUp 
system('cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && sh create_symbolic_link.sh')
% check if yarpserver is running
[a,pid]=system('pgrep yarpserver');
if(strcmp('',pid))
    system('gnome-terminal -x sh -c "yarpserver; bash"')
    pause(15)
else
    disp('yarpserver already running')
end
% check if gazebo is running
[a,pid]=system('pgrep gazebo');
if(strcmp('',pid))
    system('gnome-terminal -x sh -c "cd ~/git/learnOptimWBC/matlab/TestResults/scenarios/ && gazebo -slibgazebo_yarp_clock.so sit_icub_to_optimize_0_1.world; bash"');
    pause(15)
else
    disp('gazebo already running')
end
% check if gazebo is running
[a,pid]=system('pgrep wholeBody');
if(strcmp('',pid))
    system('gnome-terminal -x sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"')
    pause(15)
else
    disp('wholeBodyDynamicsTree already running')
end

% reset the robot in the starting position
system('gz world -r');

%% change folder (move to the folder with the simulink scheme)
name_simulink_model = 'TB_StandUp';
fullPath = which('find_simulatorIcubSim.m');
path = fileparts(fullPath);
path = strcat(path,'/',name_simulink_model);
cd(path)

%% remove the results file from the the TBstandup folder to manage the case where i did not cancel this file properly
% remove the file with all the data inside
delete ~/git/learnOptimWBC/matlab/Common/TB_StandUp/simulationResults.mat
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
    params.qfinalSitting = [70.0000001093867 0.618804962651733 0.399999267875981...
                            -67.2000001361580 34.0999961120969 4.79796140019389 43.1917772138638...
                            -67.2000001361580 34.0999961120969 4.79796140019389 43.1917772138638...
                            84.2999649174303	0.761524617074400	0.0867967193079845	-99.2529302018096	-15.8102389048266	0.0632937999425440...
                            84.2999649174303	0.761524617074400	0.0867967193079845	-99.2529302018096	-15.8102389048266	0.0632937999425440]'*(pi/180);  
    params.qfinal        = [-10   0  0, -20  30  0  45  , -20  30  0  45  , 25.5   0   0  -18.5  -5.5  0,25.5   0   0  -18.5  -5.5  0]'*(pi/180);   
    params.tswitch       = 1.5;
    % if this parameter is true we fix the desired com value(the com value is specify inside the trajectory block in the simulink)
    % if this parameter if false we optimize the com trajectory even when the robt is sitting on the bench the bench
    params.fixedcombench = false;

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
        numeric_reference_parameter{1,1}=[0.748774281834381,1.09687173841372,1.36483184164994,1.99216620455900,1.92546202844655,-0.0636699351395518,-0.00371461494356348,0.0146207387013360,-0.0167742878212803,-0.0627561462679218,0.470022984592288,0.425150445837247,0.453825179534190,0.483871810092092,0.362376650443275]'; 
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
        %% we do this smal addition to limit the joint below a fixed treshold
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
       
       user_defined_start_action =  [0.913076139695994,0.905282671038423,1.16925695886780,1.99897101764829,1.71346167883188,...
                                    -0.0252589378181975,-0.00171875837190067,0.0130805429422483,-0.0141668463935478,-0.0608148916683415,...
                                     0.468857336154740,0.433709756339607,0.442003448052191,0.482619376729153,0.360000000000000]; 
%         user_defined_start_action =    [0,0,0,0,0,...
%                                        1.5,1.5,0.0130805429422483,-0.0141668463935478,-0.0608148916683415,...
%                                         0.468857336154740,0.433709756339607,0.442003448052191,0.482619376729153,0.360000000000000]; 
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        niter = 500;  %number of generations
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