
%%%;;

%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 20;
time_struct.step = 0.001;

%% TASK PARAMETERS
%name_dat = 'sere/LBR4p5.0_scene5_UF_repellers_on_elbow__atrtactive_point_on_ee_fit5_SERE';
%name_dat = 'sere/LBR4p9.0_scene5_GHC_table_and_an_one_attractive_point_and_posture_task_SERE';
%name_dat = 'LBR4p8.0_scene9_GHC_test_wall_and_two_attractive_point';
%name_dat = 'LBR4p11.0_scene9_UF_mulitple_task_stability_Null_space_projectors';
%name_dat = 'LBR4p10.0_scene10_UF_lemniscate';
%name_dat = 'LBR4p12.0_scene0_UF_test_elastic_reference';
%name_dat = 'Jaco1.3_scene1.1';
%name_dat = 'LBR4p2.2_scene2_generalization';
%name_dat = 'lwrsimple1.0_scene_test_obs';
name_dat = 'iCub_1.0';
%name_dat = 'humanoid_bench_generator_lbrsimple_1.0';
%path=LoadParameters(name_dat);
%load(path);
%% TYPE OF CONTROLLER 
CONTROLLERTYPE ='UF';   % GHC or UF
%%

%SUBCHAIN PARAMETERS 
subchain1 =  {'r_hand','r_elbow_1','none'};
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
geom_parameters{1,1} = [0.35,-0.15,0.7]; %[0.38,-0.1,0.7]; 
geom_parameters{1,2} = [0.24,-0.23,0.7]; %[0.24,-0.23,0.7];
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

%% SCENARIO
name_scenario = 'iCub_1';%'lbr_scenario_2_gen' lbr_scenario2; %lbr_scenario5.1,'lbr_scenario9','lbr_scenario10';

%% RBT SIMULATOR PARAMETERS
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
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
    % iCub sim
    %% building initial configuration
    qi{1} = [];
    qdi{1} = [];
    list_of_kin_chain = {'trunk','left_arm','right_arm'};
    params.feet_on_ground =  [1,1];
    params.active_floating_base = false;
    params.qjInit      = bot1.InitializeState(list_of_kin_chain, params.feet_on_ground);
    params.dqjInit     = zeros(bot1.ndof,1);
    % icub starting velocity floating base
    params.dx_bInit    = zeros(3,1);
    params.omega_bInit = zeros(3,1);
    % root reference link;
    params.root_reference_link ='l_sole';
    params.tStart   = time_sym_struct.ti;
    params.tEnd     = time_sym_struct.tf;
    params.sim_step = 0.01;
    params.demo_movements = 0 ;
    params.wait     = waitbar(0,'State integration in progress...');
    params.maxtime = 100;
    params.torque_saturation = 100000;

end
%% Parameters Dependant on the type of controller

%%%EOF

switch CONTROLLERTYPE
    case 'UF'
        disp('UF_RUNTIMEPARAM')

        %%%;;

        %% PRIMARY REFERENCE PARAMETERS (this parameter only works if one of the specific trajectory has runtime parameters)
        numeric_reference_parameter{1,1} = [0.047180 0.359539 1.045565 0.374223 -0.069047 0.013630 -0.495463 -0.131683 0.668327 -0.184017 1.115775 0.884010 0.120701 0.837400 1.189048]';
        %% SECONDARY REFERENCE PARAMETERS
        secondary_numeric_reference_parameter{1,1} = []; 
        %% REPELLERS PARAMETERS
        % GENERALIZE TO MULTICHAIN !!!
        rep_obstacle_ref = [1 2]; % if i change the order of ref obstacle i change the order of repellor in the stacked case
        J_damp = 0.01;
        % with this part i choose if for each repellers i want to use 3 or one
        % activation policy if 1 only one if 0 we have three activation policy
        single_alpha_chain1 = [1 1];
        single_alpha_chain2 = [1];
        single_alpha{1} = single_alpha_chain1;
        single_alpha{2} = single_alpha_chain2;
        type_of_rep_strct={'extended_decoupled','extended_combine','stacked' };

        %% ALPHA PARAMETERS
        choose_alpha = 'RBF';  % RBF , constant, handTuned

        %RBF
        number_of_basis = 5; %5; %10; %basis functions for the RBF
        redundancy = 2; %3; %overlap of the RBF
        value_range = [0 , 12];
        precomp_sample = false;
        % value of theta that we have to change when we want to execute the result
        % from the optimization step
        %numeric_theta = [0.068017 9.937933 10.629743 8.625690 4.620175 10.724682 6.943026 1.836172 6.005996 6.404127 1.499565 5.320011 5.059803 8.438304 2.319497 8.590403 9.120348 2.400932 9.071976 6.264097 ];
        %numeric_theta =[0.068017 9.937933 10.629743 8.625690 4.620175 10.724682 6.943026 1.836172 6.005996 6.404127 1.499565 5.320011 5.059803 8.438304 2.319497 8.590403 9.120348 2.400932 9.071976 6.264097 ];
        %numeric_theta =[2.3218    2.5695    6.8006    4.6558    5.7475    8.7383    3.5058    5.2817    6.9910    6.7590    4.5235    6.3875    7.3247    6.7258 8.5637];
        a = 0; b = 0; c = 14;
        numeric_theta = [0 0 0 0 0 0 0 0 0 0 14 14 14 14 14];
        %numeric_theta = [1.34808936568616 7.65801020911318 11.4142276222402 11.0490983940875 5.40014056365278 3.15896271495000 2.49526509728241 3.38786911220609 3.59148669099237 -3.99404815876146 0.150175087891493 3.67130705179767 4.44576652255605 5.52547896983619 -5.09694541976550];
        
        % from sere 1
        %numeric_theta = [5.819383 4.412794 5.286902 7.786384 7.599614 3.512520 5.989917 9.410994 7.444834 7.472545 4.532512 5.614148 7.970080 4.498142 6.194601 6.925731 4.815911 5.490313 5.294776 6.011380 ]

        %from 10 generation of CMAES: collision with end-eff and table
        %numeric_theta = [1.351681 10.784147 9.724284 6.550806 7.740233 5.928500 8.123806 7.776163 6.548935 5.474038 7.455956 4.011111 6.704292 1.089315 3.712038 6.041540 5.098971 5.054418 6.312087 6.223340 ];

        % this is a good one (obtained by 80 generations of CMAES)
        %numeric_theta = [2.885347 7.054374 6.510485 4.220996 3.779241 7.292772 6.753379 4.039816 3.503077 7.105706 7.242047 5.176997 6.656641 7.282674 6.310105 2.320801 6.164860 5.949270 5.958774 3.349248]; 

        %numeric_theta = [2.718340 0.238570 4.959242 5.150985 10.810089 5.561797 6.436029 3.089579 7.488959 5.577574 5.300494 9.360753 5.395630 3.646393 5.427430 5.963953 10.538157 8.951330 7.672437 2.743474];
        %numeric_theta = [3.493783 6.211959 7.883578 11.988846 7.900086 9.468388 6.525209 11.867391 7.355206 8.158990 0.000000 0.054878 11.131856 8.063698 1.871041 9.107188 3.646651 8.656589 11.419753 4.346246 ];  

        %this is the task without the constraints of the table 
        %numeric_theta =[12 12 12 12 12 12 12 12 12 12];

        %constant alpha
        value1 = 0*ones(chains.GetNumTasks(1));
        values{1} = value1;
        value_range_for_optimization_routine = [-0.5 , 1.5]; % this is a trick that im using to provide bound to the optimization procedure for parametric reference


        % HandTunedAlpha
        starting_value = [0 1 0] ;
        % inf means no transition but i have to add them because im using matrix
        % so i have to keep the number of element even between each row and between each col
        t1 = [13 inf;15 inf;0.5 3];
        ti(:,:,1) = t1;
        transition_interval1 = [0.5 0.5;1 0.5;0.5 1];
        transition_interval(:,:,1) = transition_interval1;
        %% CONTROLLER PARAMETERS
        combine_rule = {'sum'}; % sum or projector (with sum reppelers are removed)
        
        % the metric change between regularized and not regularized because in the
        % regularized case i have to do N^(-1) 
        % not regularized case i have N^(-1/2)
        metric = {'M^(1)','M^(1)','M^(1)','M^(1)','M^(1)','M^(1)','M^(1)'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
        kd = [110,90,90,110,110,110,110];
        kp = [70,40,40,70,70,70,70]; % row vector one for each chain
        for i= 1:chains.GetNumChains()
            for par = 1:chains.GetNumTasks(i)
                if(strcmp(traj_type{i},'impedance'))
                    M = diag([1 1 1]);
                    D = diag([110 110 110]);
                    P =  kp(i,par)*eye(size(dim_of_task{i,par},1));
                    obj.M = M;
                    obj.D = D;
                    obj.P = P;
                    Param{i,par} = obj;
                else
                    K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
                    K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
                    obj.Kp = K_p;
                    obj.Kd = K_d;
                    Param{i,par} = obj;
                end
            end
         end
         % secondary task gains
         kd = [90,90,90,110,110,110,110];
         kp = [40,40,40,70,70,70,70]; % row vector one for each chain
         for i= 1:chains.GetNumChains()
            for par = 1:chains.GetNumTasks(i)
                 if(strcmp(traj_type_sec{i},'impedance'))
                    M = diag([1 1 1]);
                    D = diag([110 10 110]);
                    P =  kp(i,par)*eye(size(dim_of_task{i,par},1));
                    obj.M = M;
                    obj.D = D;
                    obj.P = P;
                    Param_secondary{i,par} = obj;
                else
                    K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
                    K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
                    obj.Kp = K_p;
                    obj.Kd = K_d;
                    Param_secondary{i,par} = obj;
                end
            end
         end
        % with this term i introduce a damped least square structure inside my
        % controller if regularizer is 0 i remove the regularizer action 
        % ONE FOR EACH TASK
        regularizer_chain_1 = [0 0 0 0 0 0 0 0.001]; 
        regularized_chain_2 = [1];
        regularizer{1} = regularizer_chain_1;
        regularizer{2} = regularized_chain_2;
        
        %% CONSTRAINTS PARAMETERS
        constraints_values = bot1.createConstraintsVector;
        for k = 1:2:length(constraints_values)
            constraints_functions{k} = 'LinInequality';
            constraints_functions{k+1} = 'LinInequality2';
        end
        distConstraints_values = [0.03, 0.03, 0.03, 0.03, 0.03]; % distances for the collissions constraints
        for k = 1:length(distConstraints_values)
            constraints_functions{end+1} = 'DistanceObs';
        end
        constraints_values = [constraints_values, distConstraints_values];   % vector that contains some constant that are used by the function in constraints_functions to compute the constraints_violation
        constraints_type = ones(1,length(constraints_values)); % vector that specifies if the constraints is a equality or an inequality. 1 disequality 0 equality
        
        activate_constraints_handling = true;        
        %% INSTANCE PARAMETER
        run_function = @RobotExperiment;
        fitness = @fitnessHumanoidsIcub;
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
    
        end
        %% CMAES PARAMETER
        %--- Starting value of parameters
        generation_of_starting_point = 'test'; % 'test', 'given', 'random'
        %init_parameters = 6;
        user_defined_start_action=[0 0 0 0 0 0 0 0 0 0 14 14 14 14 14];
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        niter = 480;  %number of generations
        cmaes_value_range = [-14 , 14];  % boudn that define the search space
        %cmaes_value_range{1} = [-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-0.15,-0.15,-0.15 ];  % lower bound that define the search space
        %cmaes_value_range{2} = [14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,0.15,0.15,0.15];  % upper bound that define the search space
        learn_approach = '(1+1)CMAES'; %CMAES (1+1)CMAES    
        %--- Parameter for constraints method
        method_to_use = 'vanilla';  % adaptive , vanilla , empty
        epsilon = 0.001*ones(1,length(constraints_functions)); %vector with a number of value related to the number of constraints (used only with Aaptive constraints)
        %% FITNESS PARAMETERS
        
        %%%EOF    
    case 'GHC'
        disp('GHC_RUNTIMEPARAM')

        %%%;;

        %% Constraints
        constraints_list={'vellimit','vellimit','torquelimit','torquelimit','obsavoid'}; %'obsavoid'
        cdata1 = [1;1000];
        cdata2 = [0;1000];
        cdata3 = [1;2000];
        cdata4 = [0;2000];
        cdata5 = [1;7];
        constraints_data = [cdata1, cdata2, cdata3, cdata4, cdata5];

        %% flag to choose type of alpha 
        % RBF or chained
        choose_alpha = 'chained';

        %% ChainedAlpha
        transition_interval = 1.5;

        %% Alpha RBF
        %ALPHA PARAMETERS
        %rbf
        number_of_basis = 5;
        redundancy = 2;
        value_range = [0 , 12];
        precomp_sample = false;
        % value of theta that we have to change when we want to execute the result
        % from the optimization step in MainExec
        numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

        %% Controller Parameters
        %UF and GHC   
        % GHC
        epsilon = 0.002;
        regularization = 0.01;
     
        %% CMAES PARAMETER
        % starting value of parameters
        %init_parameters = 6;
        explorationRate =0.1;%[0, 1]
        niter = 80;  %number of generations
        fitness = @fitness7;

        %% FITNESS PARAMETERS

        %%%EOF
    otherwise
        warning('Unexpected control method')
end

%% DO NOT CHANGE THIS PART!

% backup data 
rawTextFromStorage = fileread(which(mfilename));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');

% join the general static parameter with the particular static one
%rawTextFromStorage = strcat(rawTextFromStorage,rawTextFromStoragePart);

