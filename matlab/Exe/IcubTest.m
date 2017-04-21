clear variables
close all
clc

%% test selection

visualization_test = false;
simulation         = true;
%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.01;
%% Parameters for simulator
ndof = 25;
% balancing on two feet or one foot                              
% allows the visualization of torques, forces and other user-defined graphics
visualizer_graphics   =  1;                                      %either 0 or 1
visualizer_demo       =  1;                                      %either 0 or 1
visualizer_jointsPos  =  0;                                      %either 0 or 1; only if visualizer_graphics = 1
% list of kin is a way to establish the kinematic part that are inside the
% current model
%% SUBCHAIN PARAMETERS

icub = iCub('icubGazeboSim');
chain_1 = DummyRvc_iCub(icub,'l_sole');
 
subchain1 = {'com'};
target_link{1} = subchain1;

robots{1} = chain_1;
% we have to specify floating base for the icub because it gives us 
% extended dynamic matrix even if the robot is fixed base
chains = SubChains(target_link,robots,icub);

%% building initial configuration
%qi{1} = [];
%qdi{1} = [];
% list_of_kin_chain = {'trunk','left_arm','right_arm','l_sole','r_sole'};
% joints_initial_values{1,1} = [0.0  0.0  0.0];
% joints_initial_values{1,2} = [0.0  30.0  0.0  45.0  0.0 0.0 0.0];
% joints_initial_values{1,3} = [0.0  30.0  0.0  45.0  0.0 0.0 0.0];
% joints_initial_values{1,4} = [25.5   5.0    0.0  -40    -5.5  -0.1];
% joints_initial_values{1,5} = [25.5   5.0    0.0  -40    -5.5  -0.1];
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
params.qjInit      = icub.InitializeStateicubGazeboSim(params.feet_on_ground);
params.dqjInit     = zeros(icub.ndof,1);
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
params.maxtime = 50;
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
%% parameters for controller and fitness
% sitting_com:-0.120249695321353,-0.0680999719842103,0.369603821651986];
% stading_com:0.0167667444901888,-0.0681008604452745,0.503988037442802
params.xComfinal = [-0.120249695321353,-0.0680999719842103,0.369603821651986]';
% stading_pose: -10   0  0, -20  30  0  45  0, -20  30  0  45  0, 25.5   0   0  -18.5  -5.5  0, 25.5   0   0  -18.5  -5.5  0
% sitting_pose: -10   0  0, -20  30  0  45  0, -20  30  0  45  0,  90    0   0  -90    -5.5  0,  90    0   0   -90   -5.5  0
params.qfinal    = [-10   0  0, -20  30  0  45  0, -20  30  0  45  0,  90    0   0  -90    -5.5  0,  90    0   0   -90   -5.5  0]'*(pi/180);   

%% Visualization
if (visualization_test)
    icub.SetWorldFrameiCub(params.qjInit,params.dqjInit,params.dx_bInit,params.omega_bInit,params.root_reference_link);
    [~,xTb,~,~] = icub.GetState();
    chi = [xTb',params.qjInit'];
    icub.plot(chi,params);
%     figure
%     hold on;
%     root_link_pos=wbm_forwardKinematics(icub.state.w_R_b,icub.state.x_b,params.qjInit,'root_link');
%     scatter3(0,0,0,'LineWidth',100);
%     scatter3(root_link_pos(1),root_link_pos(2),root_link_pos(3));
    %icub.EnhancedPlot(params.qjInit,params);
else
    %%  REFERENCE PARAMETERS
    deg = pi/180;
    % primary trajectory
    traj_type = {'cartesian'};
    control_type = {'x'};
    type_of_traj = {'func'};
    geometric_path = {'AdHocBalance'};
    time_law = {'none'};
    %parameters first chains
                         % #basis overlap                    starting com position                                          ending com position
    geom_parameters{1,1} =  [5 ,     2 ,      -0.120249695321353,-0.0680999719842103,0.369603821651986, 0.0167667444901888,-0.0681008604452745,0.503988037442802];% sitting_com:-0.120249695321353,-0.0680999719842103,0.369603821651986];
                                                                                                                                                                  % stading_com:0.0167667444901888,-0.0681008604452745,0.503988037442802
    %geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
    dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(icub.n,1); %dim_of_task{1,4}=ones(icub.n,1);

    % secondary trajectory
    traj_type_sec = {'none'};
    control_type_sec = {'rpy'};
    type_of_traj_sec = {'func'};
    geometric_path_sec = {'fixed'};
    time_law_sec = {'linear'};
    %parameters first chains
    geom_parameters_sec{1,1} = [pi/2 0 -pi/2]; % regulation
    dim_of_task_sec{1,1}={[1;1;1]};

    %numeric_reference_parameter{1,1}=[-2.6927 -1.9295 3.0885 2.1126 1.6506 -0.0113 -0.1107 -0.0148 0.0160 -0.0510 0.4647 0.4607 0.4194 0.4561 0.3719]'; 
    numeric_reference_parameter{1,1}=[-1.8 -0.9 -0.9 +1.9 1 -0.0113 -0.001 0 0.005 0.01 0.49 0.49 0.49 0.49 0.50]'; 
    %% ALPHA PARAMETERS
    choose_alpha = 'constant';  % RBF , constant, handTune
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
    %max_time = 50; %50
    combine_rule = {'sum'}; % sum or projector (with sum reppelers are removed)
% 
%     % the metric change between regularized and not regularized because in the
%     % regularized case i have to do N^(-1) 
%     % not regularized case i have N^(-1/2)
%     metric = {'M','M'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
%     kd = 110;
%     kp = 70; % row vector one for each chain
%     for i= 1:chains.GetNumChains()
%         for par = 1:chains.GetNumTasks(i)
%             if(strcmp(traj_type{i},'impedance'))
%                 M = diag([1 1 1]);
%                 D = diag([110 110 110]);
%                 P =  kp(i,par)*eye(size(dim_of_task{i,par},1));
%                 obj.M = M;
%                 obj.D = D;
%                 obj.P = P;
%                 Param{i,par} = obj;
%             else
%                 K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
%                 K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
%                 obj.Kp = K_p;
%                 obj.Kd = K_d;
%                 Param{i,par} = obj;
%             end
%         end
% 
%      end
%      % secondary task gains
%      kd = 110;
%      kp = 70; % row vector one for each chain
% 
%      for i= 1:chains.GetNumChains()
%         for par = 1:chains.GetNumTasks(i)
%              if(strcmp(traj_type_sec{i},'impedance'))
%                 M = diag([1 1 1]);
%                 D = diag([110 10 110]);
%                 P =  kp(i,par)*eye(size(dim_of_task{i,par},1));
%                 obj.M = M;
%                 obj.D = D;
%                 obj.P = P;
%                 Param_secondary{i,par} = obj;
%             else
%                 K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
%                 K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
%                 obj.Kp = K_p;
%                 obj.Kd = K_d;
%                 Param_secondary{i,par} = obj;
%             end
%         end
%      end
%     % with this term i introduce a damped least square structure inside my
%     % controller if regularizer is 0 i remove the regularizer action 
%     % ONE FOR EACH TASK
%     regularizer_chain_1 = [0 0.001 0.001 0.001]; 
%     regularized_chain_2 = [1];
%     regularizer{1} = regularizer_chain_1;
%     regularizer{2} = regularized_chain_2;


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

    %% TODO generalize to multichain and generalize respect of controller
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
    controller = Controllers.BalanceController(chains,reference,[],[],[],[],[],[],[],[],params);

    if(simulation)
        %% simulator
        [t,q_ext,qd_ext]=DynSim_iCub(controller,params);


        %% compute fitness (for this we use a sliglty different version of the fitness function (the one with Test at the end))
        output{1} = t;
        output{2} = q_ext;
        output{3} = qd_ext;
        %% CONSTRAINTS PARAMETERS
        constraints_values = icub.createConstraintsVector;
        for k = 1:2:length(constraints_values)
            constraints_functions{k} = 'LinInequality';
            constraints_functions{k+1} = 'LinInequality2';
        end
        constraints_functions{end+1} = 'EmptyConstraints'; 
        constraints_values = [constraints_values,nan];   % vector that contains some constant that are used by the function in constraints_functions to compute the constraints_violation
        constraints_type = ones(1,length(constraints_values)); % vector that specifies if the constraints is a equality or an inequality. 1 disequality 0 equality
        activate_constraints_handling = true;
        penalty_handling=Optimization.NoPenalty(controller.GetTotalParamNum(),constraints_functions,constraints_type,constraints_values);

        fitnessHumanoidsIcubStandUpTest(output,penalty_handling,controller,params)
    
        %% Visualize forward dynamics
        figure
        hold on
        %plot3(geom_parameters{1,1}(1),geom_parameters{1,1}(2),geom_parameters{1,1}(3),'g.','MarkerSize', 30);
        %icub.plot(chi,params);
        chi = [q_ext,qd_ext];
        % get the joint position
        [state,x_b,qt_b,w_R_b,base_pose,q,dx_b,w_omega_b,qd,Nu] = icub.State(chi');
        icub.EnhancedPlot(q,params);
    end
    %% visualize trajectory (time profile and geometric trajectory)
%     s_ext = 0;
%     [p,pd,pdd,time,obj4visual]=AdHocBalanceControllerTrajectory(s_ext,time_struct,geom_parameters{1,1},'func');
%     time_vec = time_struct.ti:time_struct.step:time_struct.tf;
%     
%     s   = zeros(length(time_vec),1);
%     sd  = zeros(length(time_vec),1);
%     sdd = zeros(length(time_vec),1);
%     p_v = zeros(length(time_vec),3);
%     index = 1;
%     for t=time_struct.ti:time_struct.step:time_struct.tf
%          p_v(index,:) = obj4visual.p_real(t,numeric_reference_parameter{1,1})';
%          s(index)     = obj4visual.s(t,numeric_reference_parameter{1,1});
%          sd(index)    = obj4visual.sd(t,numeric_reference_parameter{1,1});
%          sdd(index)   = obj4visual.sdd(t,numeric_reference_parameter{1,1});
%          index = index + 1;
%     end
%     transformed_time_vec = 0:0.001:1;
%     p_test = zeros(length(transformed_time_vec),3);
%     index = 1;
%     for t = 0:0.001:1
%     p_test(index,:) = obj4visual.p_test(t,numeric_reference_parameter{1,1})';
%     index = index + 1;
%     end
%     
%     
%     figure
%     plot(time_vec,p_v(:,1))
%     figure
%     plot(time_vec,p_v(:,3))
%     %figure
%     %plot(p_v(:,1),p_v(:,3));
%     figure
%     plot(transformed_time_vec,p_test(:,1))
%     figure
%     plot(transformed_time_vec,p_test(:,3))
%     %figure
%     %plot(p_real(:,1),p_real(:,3));   
%     figure
%     plot(time_vec,s);
%     figure
%     plot(time_vec,sd);
%     figure
%     plot(time_vec,sdd);
    
    
    %% debugging preprocessor
%     fake_istance.input_4_run{1} ='icub_matlab';
%     fake_istance.input_4_run{2} = params ;
%     fake_istance.input_4_run{4} = controller;
%     [run_flag1,performance1,action1]=CheckTimeLawVelocity(fake_istance,numeric_reference_parameter{1,1});
%     [run_flag,performance,action]=StickBreaking4MonotoneTimeLaw(fake_istance,numeric_reference_parameter{1,1});
%     
    
  
end
