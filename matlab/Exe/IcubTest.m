clear variables
close all
clc

%% test selection

visualization_test = false;

%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.001;

%% Parameters for simulator
ndof = 17;
% balancing on two feet or one foot
params.feet_on_ground =  [1,1];                                  %either 0 or 1; [left,right] (in the simulator)
% allows the visualization of torques, forces and other user-defined graphics
visualizer_graphics   =  1;                                      %either 0 or 1
visualizer_demo       =  1;                                      %either 0 or 1
visualizer_jointsPos  =  0;                                      %either 0 or 1; only if visualizer_graphics = 1

params.demo_movements = 0;
% list of kin is a way to establish the kinematic part that are inside the
% current model
list_of_kin_chain = {'com','left_arm','right_arm'};

%% SUBCHAIN PARAMETERS

icub = iCub('model_arms_torso_free');
chain_1 = DummyRvc_iCub(icub,'l_sole');
 
subchain1 = {'r_gripper'};
target_link{1} = subchain1;

robots{1} = chain_1;
% we have to specify floating base for the icub because it gives us 
% extended dynamic matrix even if the robot is fixed base
chains = SubChains(target_link,robots,icub);

%% building initial configuration
%params.qjInit      = [];
params.qjInit      = zeros(icub.ndof,1);

if(~isempty(find(SubStrFind('com',list_of_kin_chain),1)))
    torsoInit    = [0.0  0.0  0.0]'; %yaw roll pitch
    %params.qjInit = [params.qjInit;torsoInit];
    string_search = {'torso_yaw','torso_roll','torso_pitch'};
    params.qjInit = icub.sortJointValue(string_search,params.qjInit,torsoInit);
end

if(~isempty(find(SubStrFind('left_arm',list_of_kin_chain),1)))
    leftArmInit  = [ -20.0  30.0  0.0  45.0  0.0 0.0 0.0]';
    %params.qjInit = [params.qjInit;leftArmInit];
    string_search = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw',...
        'l_elbow','l_wrist_prosup','l_wrist_pitch','l_wrist_yaw'};
    params.qjInit = icub.sortJointValue(string_search,params.qjInit,leftArmInit);
end

if(~isempty(find(SubStrFind('right_arm',list_of_kin_chain),1)))
    rightArmInit = [  -20.0  30.0  0.0  45.0  0.0 0.0 0.0]';
    %params.qjInit = [params.qjInit;rightArmInit];
    string_search = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw',...
        'r_elbow','r_wrist_prosup','r_wrist_pitch','r_wrist_yaw'};
    params.qjInit = icub.sortJointValue(string_search,params.qjInit,rightArmInit);
end

if(~isempty(find(SubStrFind('l_sole',list_of_kin_chain),1)) || ~isempty(find(SubStrFind('r_sole',list_of_kin_chain),1)))
    if     params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
        % initial conditions for balancing on two feet
        leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
        rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
        %params.qjInit = [params.qjInit;leftLegInit;rightLegInit];
    elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
        % initial conditions for the robot standing on the left foot
        leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
        rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]';
        %params.qjInit = [params.qjInit;leftLegInit;rightLegInit];
    elseif   params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
        % initial conditions for the robot standing on the right foot
        leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  -0.1]';
        rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
        %params.qjInit = [params.qjInit;leftLegInit;rightLegInit];
    end
    string_search = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee',...
        'l_ankle_pitch','l_ankle_roll'};
    params.qjInit = icub.sortJointValue(string_search,params.qjInit,leftLegInit);
    string_search = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee',...
        'r_ankle_pitch','r_ankle_roll'};
    params.qjInit = icub.sortJointValue(string_search,params.qjInit,rightLegInit);
end
params.qjInit      = params.qjInit*(pi/180);
params.dqjInit     = zeros(icub.ndof,1);

% icub starting velocity floating base
params.dx_bInit    = zeros(3,1);
params.omega_bInit = zeros(3,1);

% root reference link;
params.root_reference_link ='l_sole';

% specify limits
%param.limits;

% Setup integration
%plot_set

params.tStart   = time_struct.ti;
params.tEnd     = time_struct.tf;
params.sim_step = 0.01;
params.wait     = waitbar(0,'State integration in progress...');


%% Visualization

if (visualization_test)
    [~,xTb,~,~] = icub.GetState();
    chi = [xTb',params.qjInit'];
    icub.plot(chi,params)
else
    %% limit parameters
    
    [jl1,jl2]        = wbm_jointLimits();
    limits           = [jl1 jl2];
    params.limits    = limits;
    
    %%  REFERENCE PARAMETERS
    deg = pi/180;
    % primary trajectory
    traj_type = {'cartesian'};
    control_type = {'x'};
    type_of_traj = {'func'};
    geometric_path = {'fixed'};
    time_law = {'none'};
    %parameters first chains
    geom_parameters{1,1} = [0.236247320698301 -0.295336915231286 0.995201661433111]; %[0.209290598956330,0.147080124030923,0.607101111147211]; 
    %geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
    dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.n,1); %dim_of_task{1,4}=ones(bot1.n,1);

    % secondary trajectory
    traj_type_sec = {'none'};
    control_type_sec = {'rpy'};
    type_of_traj_sec = {'func'};
    geometric_path_sec = {'fixed'};
    time_law_sec = {'linear'};
    %parameters first chains
    geom_parameters_sec{1,1} = [pi/2 0 -pi/2]; % regulation
    dim_of_task_sec{1,1}={[1;1;1]};

    numeric_reference_parameter{1,1}=[]; % is not used but just to be compliant with the input structure

    %% ALPHA PARAMETERS
    choose_alpha = 'RBF';  % RBF , constant, handTune
    %RBF
    number_of_basis = 5; %5; %10; %basis functions for the RBF
    redundancy = 2; %3; %overlap of the RBF
    value_range = [0 , 12];
    precomp_sample = false;
    numeric_theta =[12 12 12 12 12];

    %constant alpha
    value1 = 1*ones(chains.GetNumTasks(1));
    values{1} = value1;
    value_range_for_optimization_routine = [-0.5 , 1.5]; % this is a trick that im using to provide bound to the optimization procedure for parametric reference


    %% CONTROLLER PARAMETERS
    max_time = 50; %50
    combine_rule = {'sum'}; % sum or projector (with sum reppelers are removed)

    % the metric change between regularized and not regularized because in the
    % regularized case i have to do N^(-1) 
    % not regularized case i have N^(-1/2)
    metric = {'M^(1)'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
    kd = 110;
    kp = 70; % row vector one for each chain
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
     kd = 110;
     kp = 70; % row vector one for each chain

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
    regularizer_chain_1 = [0 0.001 0.001 0.001]; 
    regularized_chain_2 = [1];
    regularizer{1} = regularizer_chain_1;
    regularizer{2} = regularized_chain_2;


    %%  Primary Reference
    % if type_of_task = sampled i have to specify the Time to reach the
    % end of the trajectories that is equal to the simulation time
    reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
    reference.BuildTrajs();
    reference.cur_param_set = numeric_reference_parameter; % some kind of trick to introduce parameters in the optimization procedure

    %% secondary reference 
    secondary_refs = References(target_link,traj_type_sec,control_type_sec,geometric_path_sec,geom_parameters_sec,time_law_sec,time_struct,dim_of_task_sec,type_of_traj_sec);
    secondary_refs.BuildTrajs();
    secondary_refs.cur_param_set = numeric_reference_parameter; % some kind of trick to introduce parameters in the optimization procedure

    %% activation policies

    % TODO generalize to multichain and generalize respect of controller
    if(strcmp(combine_rule,'sum'))
        number_of_action = chains.GetNumTasks(1) ;
    elseif(strcmp(combine_rule,'projector'))
        number_of_action = chains.GetNumTasks(1) + repellers.GetNumberOfWeightFuncRep(1);
    end
    %---
    switch choose_alpha
        case 'RBF'
            alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,false);       
        case 'constant'
            alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,value_range_for_optimization_routine,time_struct);
        case 'handTuned'  
            alphas = Alpha.HandTuneAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),starting_value,ti,transition_interval,time_struct);
        otherwise
            warning('Uexpected alpha functions')
    end

    %% controller
    repellers = [];
    controller = Controllers.UF(chains,reference,secondary_refs,alphas,repellers,metric,Param,Param_secondary,combine_rule,regularizer);

    %% simulator
    [t,chi,visual_param]=DynSim_iCub(controller,params);

    %% Visualize forward dynamics
    %params.wait     = waitbar(0,'Graphics generation in progress...');
    figure
    hold on
    plot3(geom_parameters{1,1}(1),geom_parameters{1,1}(2),geom_parameters{1,1}(3),'g.','MarkerSize', 30);
    icub.plot(chi,params);

  
end
