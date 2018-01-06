%% Current global configuration parameters for a specified
%  multi-task controller of a given robot:

% Configuration data for the lift-object experiment:
%
% This configuration creates and setup all necessary data types and parameters
% for the experiment with the iCub humanoid robot of lifting an object (wooden
% cube) from a deposit table to a shelf.
%
% Note: This configuration file works only if the WBM-Library is installed on
%       the system.

%%%;;

import WBM.*

% Global payload state variable for a mixed forward dynamics system
% to indicate the switchover to another forward dynamics model:
global gbl_plstate;
gbl_plstate = struct('obj_grabbed', false, 'obj_released', false, ...
                     'tidx_go', 0, 'tidx_ro', 0, 'qj_go', [], 'fval_go', []);

simulator_type = {'icub_matlab'};

% Selected multi-task controller type ('GHC' or 'UF'):
CONTROLLERTYPE = 'UF';

% Time duration structure for the experiment:
time_struct.ti   = 0;     % init. time in [s]
time_struct.tf   = 30;    % final time in [s]
time_struct.step = 0.001; % time step in [s]

% Backup file for the current set parameters:
name_dat = 'icub_atf_lift_obj_1.0';
path = LoadParameters(name_dat);
load(path);

%% Robot & simulation scenario:
%
% Initialize the model and configuration parameters and create the
% whole-body controller (WBC) with the mex-WholeBodyModel (WBM) of
% the iCub humanoid robot:
wf2fixlnk = true; % set the world frame to a fixed link
[icub_model, icub_config, ndof] = initRobotICub_liftObj();
bot1 = iCubWBC(icub_model, icub_config, wf2fixlnk);

% options for the multi-chain tree model ...
opt = struct('name', '', 'manufacturer', '', 'comment', '', ...
             'ee_links', [], 'plotopt3d', []);
opt.comment = 'Chain-tree model of the iCub for the lift-obj. simulation.';

% Create the multi-chain tree model (interface) of the iCub robot and
% add it to the robot list of the learning-framework:
chn_tree_bot1 = MultChainTreeICub(bot1, 'l_sole', opt);
robots        = {chn_tree_bot1};

% Initialize the environment scenario for the simulation:
show_light = true;
sim_config = initSimScenario_liftObj(bot1.hwbm, 'DarkScn', show_light);

name_scenario = 'icub_lift_obj';

%% Elementary tasks:

% Link names for the elementary tasks:
%
% Subchain array (array of links) with the end-effectors (ee) and other links of
% a given kinematic chain tree which will be controlled by the system:
%
% Note: Each kinematic chain can have only one subchain array with at least one
%       element and the array must have at least one end-effector. Further links
%       or joint positions to be controlled by the system, can be added to the
%       array, in dependency of the given elementary tasks (one link for each
%       elementary task) of the specified learning problem.
%
% elem. task idx:    1         2          3            4            5            6          7
% target pos.   :    A         B          C            D            E            F
subchain1   =   {'l_hand', 'r_hand', 'l_gripper', 'r_gripper', 'l_elbow_1', 'r_elbow_1' , 'none', ...
                 'l_hand', 'r_hand', 'l_gripper', 'r_gripper', 'l_elbow_1', 'r_elbow_1' }; % Cartesian space and/or joint space
%                    G         H          I            J            K            L
%                    8         9         10           11           12           13

% subchain1 = {'l_hand', 'r_hand', 'l_gripper', 'r_gripper', 'l_elbow_1', 'r_elbow_1' , 'none'};
% subchain1 = {'l_hand', 'r_hand', 'l_gripper', 'r_gripper', 'none'};

target_link = {subchain1}; % add the array to the target links of the control system ...

nTsk  = size(subchain1,2); % number of tasks
ip_jt = 7;                 % index position of the joint task
% ip_jt = 5;

% Target points for the elementary tasks:
%
% Cartesian coordinates, elementary target points to be reached, for the left
% and right trajectory of each joint controlled by the system to achieve the
% desired learning behavior:
trg_pts = cell(1,nTsk);

% side length of the cube to be grabbed ...
l_s  = sim_config.environment.vb_objects(3,1).dimension(1,1);
ls_h = l_s * 0.5; % half side length

% High-level commands:
% (‡) ... elementary tasks
%
%   * GRAB_OBJECT_AT_POS(A,B)
%       + [C,D] = ESTIM_POS_GRIPPERS(A,B)
%       + [E,F] = ESTIM_POS_ELBOWS(A,B)
%
%       + MOVE_ELBOWS_TO_POS(E,F)
%           - MOVE_L_ELBOW_TO_POS(E)       (‡)
%           - MOVE_R_ELBOW_TO_POS(F)       (‡)
%       + MOVE_HANDS_TO_POS(A,B)
%           - MOVE_L_HAND_TO_POS(A)        (‡)
%           - MOVE_R_HAND_TO_POS(B)        (‡)
%       + MOVE_GRIPPERS_TO_POS(C,D)
%           - MOVE_L_GRIPPER_TO_POS(C)     (‡)
%           - MOVE_R_GRIPPER_TO_POS(D)     (‡)
%
%   * SET_JOINT_POSITIONS(jnt_pos)         (‡)
%
%   * MOVE_OBJECT_TO_POS(G,H)              % move obj. to target pos.
%       + [I,J] = ESTIM_POS_GRIPPERS(G,H)
%       + [K,L] = ESTIM_POS_ELBOWS(G,H)
%
%       + MOVE_ELBOWS_TO_POS(K,L)
%           - MOVE_L_ELBOW_TO_POS(K)       (‡)
%           - MOVE_R_ELBOW_TO_POS(L)       (‡)
%       + MOVE_HANDS_TO_POS(G,H)
%           - MOVE_L_HAND_TO_POS(G)        (‡)
%           - MOVE_R_HAND_TO_POS(H)        (‡)
%       + MOVE_GRIPPERS_TO_POS(I,J)
%           - MOVE_L_GRIPPER_TO_POS(I)     (‡)
%           - MOVE_R_GRIPPER_TO_POS(J)     (‡)

sh_h_tbl  = [0  0  0.20]; % height from UE table to UE shelf (UE ... upper edge)
el2_d_el1 = [0  0  0.08]; % distance from elbow pos. 1 to elbow pos. 2
g_d_h     = 0.0645;       % distance from hand frame {h} to gripper frame {g}

% GRAB_OBJECT_AT_POS:
% - MOVE_HANDS_TO_POS:
trg_pts{1,1} = [0.20   ls_h  (0.50+ls_h)]; % move l_hand to targ. pos. A
trg_pts{1,2} = [0.20  -ls_h  (0.50+ls_h)]; % move r_hand to targ. pos. B
% - MOVE_GRIPPERS_TO_POS:
trg_pts{1,3} = [(0.20+g_d_h)   ls_h  (0.50+ls_h)]; % move l_gripper to sub-targ. pos. C
trg_pts{1,4} = [(0.20+g_d_h)  -ls_h  (0.50+ls_h)]; % move r_gripper to sub-targ. pos. D
% - MOVE_ELBOWS_TO_POS:
trg_pts{1,5} = [0.10   0.13  0.63]; % move l_elbow_1 to sub-targ. pos. E
trg_pts{1,6} = [0.13  -0.17  0.63]; % move r_elbow_1 to sub-targ. pos. F

% SET_JOINT_POSITIONS:
trg_pts{1,7} = repmat(0.5, 1, ndof); % joint positions to minimize the torques in the fitness function.
% trg_pts{1,5} = repmat(0.5, 1, ndof);

% MOVE_OBJECT_TO_POS (final target points):
% - MOVE_HANDS_TO_POS:
trg_pts{1,8}  = trg_pts{1,1} + sh_h_tbl;  % move l_hand to final pos. G
trg_pts{1,9}  = trg_pts{1,2} + sh_h_tbl;  % move r_hand to final pos. H
% - MOVE_GRIPPERS_TO_POS:
trg_pts{1,10} = trg_pts{1,3} + sh_h_tbl;  % move l_gripper to sub-targ. pos. I
trg_pts{1,11} = trg_pts{1,4} + sh_h_tbl;  % move r_gripper to sub-targ. pos. J
% - MOVE_ELBOWS_TO_POS:
trg_pts{1,12} = trg_pts{1,5} + el2_d_el1; % move l_elbow_1 to sub-targ. pos. K
trg_pts{1,13} = trg_pts{1,6} + el2_d_el1; % move r_elbow_1 to sub-targ. pos. L

%% Reference parameters:

% Geometric parameter array for every trajectory type
% (functional or sampled) of an elementary task:
geom_parameters = trg_pts;

% Dimension selection array to specify which Cartesian dimension or
% joint of every elementary task should be controlled by the system:
%    - 1 ... dimension/joint active
%    - 0 ... dimension/joint not active
dim_of_task = cell(1,nTsk);
%            dimension:  x  y  z
dim_of_task(1,:)     = {[1; 1; 1]};  % activated dimensions
dim_of_task{1,ip_jt} = ones(ndof,1); % activated joints

% Parameters for the primary trajectories:
%
% Array of trajectory-controller types (one controller for each elementary task):
%    - cartesian     (UF)  ... Cartesian controller (for coordinates)
%    - impedance     (UF)  ... impedance controller
%    - cartesian_x   (GHC) ... Cartesian controller to control only the translation
%    - cartesian_rpy (GHC) ... Cartesian controller to control only the rotation in Euler angles (ZYX)
%    - joint    (GHC & UF) ... joint position controller
traj_type      = cell(1,nTsk);
traj_type(1,:) = {'cartesian'}; traj_type{1,ip_jt} = 'joint';

% Control type array (one type for each elementary task) to specify
% what should be controlled by the trajectory-controller:
%    - regulation (GHC) ... use controller to reach a point
%    - tracking   (GHC) ... use controller to follow a trajectory
%    - x           (UF) ... use Cartesian controller to control only the translation
%    - rpy         (UF) ... use Cartesian controller to control only the rotation in Euler angles (ZYX)
%    - none  (GHC & UF) ... if the controller has no parameters
control_type      = cell(1,nTsk);
control_type(1,:) = {'x'}; control_type{1,ip_jt} = 'none';

% Type of trajectory to be generated for each elementary task:
%    - func (functional) ... closed form trajectory, a dynamic function with
%                            different time steps for each generation
%    - sampled           ... sampled trajectory for tracking, a constant function
%                            with fixed time steps for all generations
%    - none              ... for regulation, no trajectory will be generated
type_of_traj      = cell(1,nTsk);
type_of_traj(1,:) = {'sampled'};

% Type of trajectory path to follow for an elementary tracking task:
%    - rectilinear ... rectilinear trajectory path with time law
%    - lemniscate  ... lemniscate trajectory path with time law
%    - circular    ... circular trajectory path with time law
%    - elastic     ... dynamic trajectory path with time law that
%                      will change at every generation
%    - fixed       ... trajectory path of a given fixed point
%    - sampled     ... sampled trajectory path
%    - ball        ... trajectory path of a given fixed ball
%    - none        ... if an elementary regulation task is given
geometric_path      = cell(1,nTsk);
geometric_path(1,:) = {'fixed'};

% Type of time law to be used for each trajectory path of
% an elementary tracking task:
%    - exponential ... exponential time law function
%    - linear      ... linear time law function
%    - trapezoidal ... trapezoidal time law function (deactivated)
%    - none        ... if an elementary regulation task is given
%
% Note: The time law specifies the time restriction (tstart, tfinish)
%       on every subchain.
time_law      = cell(1,nTsk);
time_law(1,:) = {'none'};
%time_law(1,:) = {'exponential'};
% END of Parameters for the primary trajectories.

% configure the link trajectories:
idx_j = 7; % (*)
%             link name     |  joint annot. pos.  | mkr |      traj. color
traj_conf = { subchain1{1,2}, {'left_arm',  idx_j}, 'o', WBM.wbmColor.forestgreen; ... % l_hand
              subchain1{1,1}, {'right_arm', idx_j}, 'o', WBM.wbmColor.tomato };        % r_hand
% (*) ... joint index

% configure for the controlled links (tasks)
% their target points to be reached:
trg_color1 = WBM.wbmColor.turquoise1;
trg_color2 = WBM.wbmColor.orangered;
trg_color3 = WBM.wbmColor.deeppink;
trg_conf = { trg_pts{1,2}, '+', trg_color1; trg_pts{1,1}, '+', trg_color1; ...   % first targets for the hands (at object)
             trg_pts{1,4}, 'o', trg_color2; trg_pts{1,3}, 'o', trg_color2; ...   % 1st sub-targets for the grippers (at object)
             trg_pts{1,6}, 'o', trg_color2; trg_pts{1,5}, 'o', trg_color2; ...   % 1st sub-targets for the elbows
             trg_pts{1,9}, '+', trg_color3; trg_pts{1,8}, '+', trg_color3; ...   % final targets for the hands (goal position of obj.)
             trg_pts{1,11}, 'o', trg_color2; trg_pts{1,10}, 'o', trg_color2; ... % 2nd sub-targets for the grippers
             trg_pts{1,13}, 'o', trg_color2; trg_pts{1,12}, 'o', trg_color2; };  % 2nd sub-targets for the elbows

% trg_conf = { trg_pts{1,2}, '+', trg_color3; trg_pts{1,1}, '+', trg_color3; ...
%              trg_pts{1,4}, 'o', trg_color2; trg_pts{1,3}, 'o', trg_color2; };

% create and setup the trajectory and target point objects for the hands
% and other links and add them to the simulation configuration structure:
[lnk_traj, trg_pts] = setupTrajectories_liftObj(traj_conf, trg_conf);
sim_config.trajectories = lnk_traj;
sim_config.target_pts   = trg_pts;
bot1.sim_config         = sim_config;

% Create a kinematic chain model from the target links of the iCub robot:
% Note: The chain model will be controlled only by the UF-controller.
chains = SubChains(target_link, robots, bot1);
nChns  = chains.GetNumChains();

% Parameters for the secondary trajectories:
% Note: The secondary trajectories are only needed
%       in experiments with one-armed robots.
%
% The secondary trajectories are not needed for the simulations
% with the iCub robot or other multi-chain tree robots.
% Thus, create some dummy parameters ...
empty_arr = cell(1,nTsk);
geom_parameters_sec = {};
dim_of_task_sec     = {};
traj_type_sec       = empty_arr; traj_type_sec(1,:) = {'none'};
control_type_sec    = empty_arr;
type_of_traj_sec    = empty_arr;
geometric_path_sec  = empty_arr;
time_law_sec        = {};
% END of Parameters for the secondary trajectories.

% Static parameters for the specified multi-task controller:
switch CONTROLLERTYPE
    case 'UF'
        % repeller parameters (scenario dependent):
        rep_subchain      = 7;
        rep_target_link   = {rep_subchain};
        rep_type          = {'cartesian_x'};
        rep_mask          = {[1  1  1]};
        rep_type_of_J_rep = {'DirectionCartesian'};

        chain_dof = zeros(1,nChns);
        for i = 1:nChns
            chain_dof(1,i) = chains.GetNumLinks(i);
        end
        UF_StaticParameters
    case 'GHC'
        GHC_StaticParameters
    otherwise
        warning('Unexpected control method')
end

%% Simulator parameters for the iCub robot:

% set the time values for the simulator ...
time_sym_struct = time_struct;

% iCub simulation - initial parameter configuration:
qi  = {[]}; % initial joint positions of the robot (empty = undefined)
qdi = {[]}; % initial joint velocities of the robot (empty = undefined)

% Chain list for the arms-and-torso free model of the iCub:
% Note: This chain list is not defined in the joint configurations
%       of the Yarp-WholeBodyInterface [1], but its components are
%       defined in the iCub Model Naming Conventions:
%       <http://wiki.icub.org/wiki/ICub_Model_naming_conventions>
%
%       The subchain 'trunk' is an alias name for the
%       Yarp control board name 'torso'.
%
% Source:
%   [1] Yarp-WholeBodyInterface: <https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface.ini>
%
list_of_kin_chain = {'trunk', 'left_arm', 'right_arm'};

% Initial joint positions (in [deg]) for
% the specified chains of the iCub robot:
joints_initial_values = cell(1,3);
joints_initial_values{1,1} = icub_config.jpos_torso.';
joints_initial_values{1,2} = icub_config.jpos_left_arm.';
joints_initial_values{1,3} = icub_config.jpos_right_arm.';

params.feet_on_ground = [1 1];
params.numContacts    = sum(params.feet_on_ground,2);

if (params.feet_on_ground(1,1) && params.feet_on_ground(1,2))
    % contact constraints for 2 feet on ground
    params.contactLinkNames = {'l_sole', 'r_sole'};
elseif (params.feet_on_ground(1,1) && ~params.feet_on_ground(1,2))
    % contact constraints for left foot on ground
    params.contactLinkNames = {'l_sole'};
elseif (~params.feet_on_ground(1,1) && params.feet_on_ground(1,2))
    % contact constraints for right foot on ground
    params.contactLinkNames = {'r_sole'};
end

% since the legs of the current model are stiffened, the floating base
% of the robot must be deactivated else, the simulation will fail (the
% robot will fall down onto the ground) ...
params.active_floating_base = false;

% setup the initial body pose (joint positions) ...
params.qjInit = InitializeState(bot1, list_of_kin_chain, params.feet_on_ground, joints_initial_values);

% special initialization (direct coordinates in joint space),
% i.e. to simulate that the object is already grabbed ...
% params.qjInit = vertcat(0.000306538410521390, 1.48545790636447, 0.965139521949186, 0.519272065537346, -0.308698523857037, 0.777660970642602, ...
%                         0.806265900705644, -0.348192373542131, 1.08676227465339, 1.21493777377343, 0.849693769907642, -0.670115697765184, ...
%                         0.191954643266296, 1.18836007657384, -0.346779186660574, 0.307505236736724, 0.0622921720016784); % for 17-DoF (works well)

% params.qjInit = vertcat(0.000743032436796784, 0.483505561076664, 0.483505130390210, 0.483505090995381, 1.48590762243836, 0.966287578484464, ...
%                         0.519825860791593, 0.483505073115889, 0.483505081848021, 0.483505145402124, -0.307745282627135, 0.777661637630826, ...
%                         0.805176160499207, 0.483505103059566, -0.347796114998517, 0.483505075636575, 0.483505036434137, 0.483505090878814, ...
%                         0.483505091968368, 1.08646333690640, 1.21570797363058, 0.850318308887626, 0.483505092462746, 0.483505092350754, ...
%                         0.483505090529302, -0.670597439794760, 0.191690222834047, 1.18907369480943, 0.483505091095043, -0.345356114103543, ...
%                         0.307598082795097, 0.0620674035615882); % for 32-DoF
%                         % Note: The given policy torque controller of the learnOptimWBC-toolbox
%                         %       works incorrect with the iCub-model of 32 DoF.

params.dqjInit     = zeros(bot1.ndof,1); % initial angular joint velocities
params.dx_bInit    = zeros(3,1);         % initial linear base velocity (Cartesian velocity)
params.omega_bInit = zeros(3,1);         % initial angular base velocity

% Define the fixed reference link for the world frame (WF):
if params.feet_on_ground(1,1)
    params.root_reference_link = 'l_sole';
else
    params.root_reference_link = 'r_sole';
end

% the values of these parameters depending on the chosen controller type ...
params.tStart   = time_struct.ti;
params.tEnd     = time_struct.tf;
params.sim_step = 0.01; % if sim_step > time_struct.step, the simulation will run faster
%params.sim_step = time_struct.step;
params.maxtime  = 1000; %500; %150; %100;
%params.wait     = waitbar(0, 'State integration in progress...');

params.demo_movements    = false;
params.torque_saturation = 100000;

% Use mixed forward dynamics models:
%   Enable this flag if the controller should use
%   different forward dynamic methods during the
%   simulation (e.g. forward dynamics with and
%   without payload).
params.mixed_fd_models = true;

% Data structure for the input arguments of the
% forward dynamics functions of the WBM-Library:
% Note: The values of these input arguments
%       depending on the given situation.
%
% input arguments for the lift-object experiment:
mu_s = 1; %0.5;              % static friction coefficient for surfaces
ac_f = 0;                    % acceleration of both feet
foot_contact = [true, true]; % both feet are in contact with the ground
hand_contact = [true, true]; % both hands will be in contact with the object to lift

params.fdyn_data = setupFDynData(bot1, foot_contact, hand_contact, ac_f, mu_s);

% Constraint links for the fitness function:
% link order: [<links of the left arm>, <head link>, <links of the right arm>].
% arm link order: bottom up, [x_gripper/x_hand, ..., x_shoulder_1], x ... r/l.
cstr_lnk_names = { 'l_gripper', 'l_hand', 'l_wrist_1', 'l_elbow_1', 'l_shoulder_1', 'head', ...
                   'r_gripper', 'r_hand', 'r_wrist_1', 'r_elbow_1', 'r_shoulder_1' };

nCLnks = size(cstr_lnk_names,2); % number of constraint links

% Index positions of the target points for the elementary tasks:
% Note: The indices of the target points must be in the same
%       order as the given links for the elementary tasks
%       which are listed in 'subchain1'.
%       Only the target indices of tasks in Cartesian space
%       are allowed to be set in this index list.
idx_tp        = 1:nTsk;
idx_tp(ip_jt) = []; % remove index of the joint task

% Settings for the fitness function:
fset.smp_rate        = 10;   % sample rate
fset.tlim            = 30;   % time limit (in seconds)
fset.eps             = 1e-2; %1e-3 % tolerance value epsilon
fset.intrpl_step     = 1e-3; % interpolation step
fset.max_effort      = 3.5e+5;
fset.max_traj_err    = 250;
fset.weight_effort   = 1;
fset.weight_traj_err = 3;

% set the input arguments for the fitness function:
params.fit_argin = {cstr_lnk_names, idx_tp, fset};
% END of Reference parameters.

%%%EOF

%% Multi-Task Controllers:
%  Note: The parameters depending on the selected controller type.
switch CONTROLLERTYPE
    case 'UF'
        disp('UF_RUNTIMEPARAM')

        %%%;;

        % Primary reference parameters:
        % Note: The reference parameter array works only if at least one of the
        %       specified trajectories has some runtime parameters given.
        numeric_reference_parameter = {repmat(0.4, 15, 1)};

        % Secondary reference parameters:
        secondary_numeric_reference_parameter = {[]};

        % Repeller parameters:
        %
        % Index vector to link each obstacle with a repeller object:
        % Note: If the index order of the given obstacles is changed, then the
        %       index order to the linked repeller objects must be also changed
        %       to the same order.
        rep_obstacle_ref = [1 2 3]; % 1 ... deposit table, 2 ... shelf, 3 ... wooden cube (object to grab)

        J_damp = 0.01;

        % Activation policy:
        %    - 1 ... only one activation policy for each repeller object
        %    - 0 ... 3 activation policies for each repeller object
        single_alpha_chain1 = [1 1 1];
        single_alpha_chain2 = 1;

        single_alpha{1,1} = single_alpha_chain1;
        single_alpha{1,2} = single_alpha_chain2;
        type_of_rep_strct = {'extended_decoupled', 'extended_combine', 'stacked'};
        % END of Repeller parameters.

        % Alpha parameters:
        %
        % Activation policy representation:
        choose_alpha = 'RBF'; % 'RBF' (default), 'constant' or 'handTuned'

        % RBF-Network:
        number_of_basis = 5; % number of radial basis functions for the network
        redundancy      = 2; % overlap of the RBF
        value_range     = [0 12];
        precomp_sample  = false;

        % Initial time profile array to define the theta values of each
        % activation policy of the elementary tasks:
        %   The theta values are important for computing the
        %   task-priorities.
        %
        % Note: To execute the results from the optimization step, the
        %       theta values in 'MainExec' must be replaced with the
        %       optimized values that are calculated in 'MainOptRobust'.
        %
        %       The time profile will not used if the 'MainOptRobust' procedure
        %       is called instead of the 'MainExec'.
        nb = number_of_basis;
        v1 = ones(1,nb);

        numeric_theta_opt = [];

        if ~isempty(numeric_theta_opt)
            numeric_theta = numeric_theta_opt;
        else
            numeric_theta = zeros(1,nb*nTsk);

            theta = [8, 8, 14, 14, 6, 6, 1, 0, 0, 0, 0, 0, 0];   % only grabbing (second part (lifiting) deactivated)
            % theta = [0, 0, 0, 0, 0, 0, 1, 8, 8, 12, 12, 6, 6];   % only lifting (first part (grabbing) deactivated)
            % theta = [6, 6, 12, 12, 5, 5, 1, 5, 5, 10, 10, 4, 4]; % grabbing & lifting
            % theta = [8, 8, 14, 14, 6, 6, 1, 8, 8, 14, 14, 6, 6];

            % theta = [8, 8, 14, 14, 6, 6, 0.8];

            % theta = [12, 12, 14, 14, 3];
            % theta = [7, 7, 14, 14, 1.4];

            j = 1;
            for i = 1:nTsk
                numeric_theta(1,j:(i*nb)) = v1*theta(1,i);
                j = j + nb;
            end
        end

        % Constant alpha:
        value1 = zeros(chains.GetNumTasks(1));
        values = {value1};
        value_range_for_optimization_routine = [-0.5  1.5]; % this is a trick to provide bound to the optimization
                                                            % procedure for the parametric reference ...
        % Hand-tuned alpha:
        starting_value = [0 1 0];

        % Transition interval matrices:
        % Time length of each transition interval.
        %    - inf ... infinity --> no transition
        t1        = [13  inf; 15  inf; 0.5  3];
        ti(:,:,1) = t1;

        transition_interval1       = [0.5  0.5; 1  0.5; 0.5  1];
        transition_interval(:,:,1) = transition_interval1;
        % END of Alpha parameters.

        % Parameters for the UF-controller:
        %
        % Combine rule:
        % Note: If 'sum' is set, then the repeller objects will not be used.
        combine_rule = {'sum'}; % 'sum' or 'projector'

        % Metric change list:
        % The list describes the metric change of each task between the
        % regularized and the not regularized case of the controller:
        %    - N^(-1)   ... for the regularized case
        %    - N^(-1/2) ... for the not regularized case
        %
        % e.g. if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2).
        metric = cell(1,nTsk); metric(1,:) = {'M^(1)'};

        % Primary task gains:
        kd = repmat(80, 1, nTsk); % one gain value for each task
        kp = repmat(30, 1, nTsk);

        for i = 1:nChns
            for par = 1:chains.GetNumTasks(i)
                if strcmp(traj_type{1,i}, 'impedance')
                    obj.M = diag([1 1 1]);
                    obj.D = diag([110 110 110]);
                    obj.P = kp(i,par) * eye(size(dim_of_task{i,par},1));
                    Param{i,par} = obj;
                else
                    obj.Kp = kp(i,par) * eye(size(dim_of_task{i,par},1));
                    obj.Kd = kd(i,par) * eye(size(dim_of_task{i,par},1));
                    Param{i,par} = obj;
                end
            end
        end

        % Secondary task gains:
        % Note: If the secondary trajectories are not used, then the
        %       secondary task gains will also not be used.
        kp = [];
        kd = [];
        if ~isempty(geom_parameters_sec)
            if (isempty(kp) || isempty(kd))
                error('The gain values are undefined!')
            end

            for i = 1:nChns
                for par = 1:chains.GetNumTasks(i)
                    if strcmp(traj_type_sec{1,i}, 'impedance')
                        obj.M = diag([1 1 1]);
                        obj.D = diag([110 10 110]);
                        obj.P = kp(i,par) * eye(size(dim_of_task{i,par},1));
                        Param_secondary{i,par} = obj;
                    else
                        obj.Kp = kp(i,par) * eye(size(dim_of_task{i,par},1));
                        obj.Kd = kd(i,par) * eye(size(dim_of_task{i,par},1));
                        Param_secondary{i,par} = obj;
                    end
                end
            end
        else
            Param_secondary = {};
        end

        % Task regulation for each chain:
        %  - 0    ... normal UF (no regulation)
        %  - != 0 ... activate regularized UF
        %
        % Note: If values of the chain regulation terms are different from zero,
        %       then the nonzero terms transform the UF into a regularized UF by
        %       activating a damped least square method (regularization) inside
        %       of the controller.
        %
        %       If the regulation method is deactivated, i.e. all values are set
        %       to zero, the calculation of the simulation is much faster. This
        %       is very helpful in the testing phase of an experiment.
        %
        %       The regularizer term is an array of vectors with chain regulation
        %       values. The size of each vector is the number of tasks of the
        %       current chain, i.e. one regulation value for each task.
        %
        % Primary chain:
        regularizer_chain_1 = zeros(1,nTsk);
        % d = 0.001; % damp value
        % regularizer_chain_1(1,1) = d; % task 1
        % regularizer_chain_1(1,2) = d; % task 2
        % regularizer_chain_1(1,6) = d; % task 6
        % regularizer_chain_1(1,7) = d; % task 7

        % Secondary chain:
        regularized_chain_2 = 1;

        regularizer = cell(1,2);
        regularizer{1,1} = regularizer_chain_1;
        regularizer{1,2} = regularized_chain_2;
        % END of Parameters for the UF-controller.

        % Constraint parameters:
        %
        constraints_values     = createConstraintsVector(bot1);
        distConstraints_values = ones(1,nCLnks) * 0.03; % distances for the collision constraints
        nICs = length(constraints_values);     % number of inequality constraints
        nDCs = length(distConstraints_values); % number of distance constraints
        nFDCs = 0; % number of fixed distance constraints (depends on the given fitness function)
        % nFDCs = 3;

        nCVals = nICs + nDCs + nFDCs; % total number of constraint values

        constraints_functions = cell(1,nCVals);

        % Inequality constraint functions for the fitness function to compute
        % the constraint violations (angular position constraints):
        for k = 1:2:nICs
            constraints_functions{1,k}   = 'LinInequality';  % superior constraint
            constraints_functions{1,k+1} = 'LinInequality2'; % inferior constraint
        end

        % Distance constraint functions for the collision detection:
        for k = 1:nDCs
            constraints_functions{1,nICs+k} = 'DistanceObs'; % collision detection constraint
        end

        % Penalty functions to a fixed distance (with a tolerance of epsilon):
        if (nFDCs > 0)
            for k = 1:nFDCs
                constraints_functions{1,nICs+nDCs+k} = 'FixedDistanceEquality';
            end
        end

        % side length of the wooden cube:
        ls_cub = bot1.sim_config.environment.vb_objects(3,1).dimension(1,1);

        % set the fixed distance for the fixed distance constraints:
        fixDistance = ls_cub + 0.1; % ls_cub/2 + 5 cm (on both sides from the CoM)

        % Constraint values (one constant value for each constraint function):
        constraints_values = horzcat(constraints_values, distConstraints_values, ...
                                     repmat(fixDistance, 1, nFDCs));

        % Constraint types:
        % Vector that specifies if the constraints are equality or inequality functions.
        %    - 1 ... inequality function
        %    - 0 ... equality function
        constraints_type = ones(1,nICs);
        constraints_type(1,nICs) = 0; % the last constraint is an equality function

        activate_constraints_handling = true;
        % END of Constraint parameters.

        % Instance parameters:
        %
        % Function handle to a specified forward dynamics simulation function:
        % Note: This function will be executed for every optimization problem
        %       (experiment) with the given robot.
        run_function = @RobotExperiment;

        % Function handle to a specified fitness function for the
        % optimization method (CMA-ES) - only for MainOptRobust:
        % The fitness function forces the optimizer to learn the
        % desired behavior for the robot.
        fitness = @fitnessICubLiftObj;

        % Function handle to a specified cleanup function to erase the
        % controller data (torques, time, etc.) of the current experiment:
        clean_function = @RobotExperimentCleanData;

        % Set the input parameters for the optimizer and
        % for the forward dynamics simulation:
        % Note: input{4} is reserved for the controller object.
        %       The controller will be set in the Init-function.
        input = cell(1,5);
        input{1,1} = simulator_type{1,1}; % rbt/v-rep
        input{1,2} = params;
        input{1,3} = time_sym_struct;
        input{1,5} = fixDistance;
        % END of Instance parameters.

        % CMA-ES parameters:
        %
        % Set the type of start points for the initial generation:
        %    - test   ... initialization vector with user defined start points
        %    - given  ... initialization vector with a given constant value for each start point
        %    - random ... initialization vector of random generated values within a given range as start points
        generation_of_starting_point = 'test';

        % init_parameters = 6; % initial value for the optimization
                               % (the scale is between 0-14, so 6 = 0.5)

        user_defined_start_action = zeros(1,nb*nTsk);
        % start point of each numeric theta (one theta value for each task):
        theta_st = repmat(5, 1, nTsk); theta_st(1,ip_jt) = 3;

        j = 1;
        for i = 1:nTsk
            user_defined_start_action(1,j:(i*nb)) = v1*theta_st(1,i);
            j = j + nb;
        end

        % Parameters for MainOptRobust:
        explorationRate = 0.15; %0.15; %0.2; %0.1; %[0 1];
        %explorationRate = 0.5;

        niter = 100; % number of iterations (generations)
        % END of Parameters for MainOptRobust.

        % Set the value range (search space) for the CMA-ES algorithm:
        % Note: The value range will be used if the start points for
        %       the initial generation are set to 'random'.
        cmaes_value_range = cell(1,2);
        cmaes_value_range{1,1} = [-14  14]; % lower bound of the search space
        cmaes_value_range{1,2} = [];        % upper bound of the search space

        % Learning approach (type of CMA-ES algorithm to be used):
        %    - CMAES      ... normal CMA-ES algorithm
        %    - (1+1)CMAES ... (1+1)-CMA-ES with covariance constrained adaption
        learn_approach = '(1+1)CMAES';

        % Parameters for the additional constraint method to be used:
        %    - adaptive ... use CMA-ES with adaptive constraints
        %    - vanilla  ... use CMA-ES with vanilla constraints
        %    - empty    ... use CMA-ES without any constraints
        method_to_use = 'vanilla';

        if strcmp(method_to_use, 'adaptive')
            % vector of epsilon values (one for each constraint function) ...
            epsilon = ones(1,nCVals) * 0.001;
        end
        % END of CMA-ES parameters.

        % Fitness parameters:

        % END of Fitness parameters.
        %%%EOF
    case 'GHC'
        disp('GHC_RUNTIMEPARAM')

        %%%;;

        % Constraints:
        constraints_list = {'vellimit', 'vellimit', 'torquelimit', 'torquelimit', 'obsavoid'};
        cdata1 = [1; 1000];
        cdata2 = [0; 1000];
        cdata3 = [1; 2000];
        cdata4 = [0; 2000];
        cdata5 = [1; 7];
        constraints_data = horzcat(cdata1, cdata2, cdata3, cdata4, cdata5);

        % Alpha parameters:
        %
        % Alpha type (activation policy representation):
        choose_alpha = 'chained'; % 'RBF' or 'chained'

        % Chained alpha:
        transition_interval = 1.5;

        % RBF-Network:
        number_of_basis = 5;
        redundancy      = 2;
        value_range     = [0 12];
        precomp_sample  = false;

        % Initial time profile array to define the theta values of each
        % activation policy of the elementary tasks:
        % Note: To execute the result from the optimization step in 'MainExec'
        %       the theta values must be changed to the optimized values.
        %
        %       The time profile will not be used if the 'MainOptRobust'
        %       procedure is called.
        nb = number_of_basis;
        v1 = ones(1,nb);

        numeric_theta_opt = [];

        if ~isempty(numeric_theta_opt)
            numeric_theta = numeric_theta_opt;
        else
            numeric_theta = zeros(1,nb*nTsk);
            %        A  B   C   D  E  F     G  H  I  J  K  L
            theta = [8, 8, 14, 14, 6, 6, 1, 0, 0, 0, 0, 0, 0];

            j = 1;
            for i = 1:nTsk
                numeric_theta(1,j:(i*nb)) = v1*theta(1,i);
                j = j + nb;
            end
        end

        % Parameters for the GHC-controller:
        epsilon        = 0.002;
        regularization = 0.01;

        % CMA-ES parameters:
        explorationRate = 0.1; %0.5; %[0 1];
        niter           = 80;        % number of iterations (generations)
        fitness         = @fitness7; % fitness function for the CMA-ES algorithm

        % init_parameters = 6; % initial value for the optimization
        %                      % (the scale is between 0-14, so 6 = 0.5)

        % Fitness parameters:

        % END of Fitness parameters.
        %%%EOF
    otherwise
        warning('Unexpected control method')
end

%% DO NOT CHANGE THIS PART!

% Data backup:
rawTextFromStorage = fileread(which(mfilename));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'], 'match', 'once');

% join the general static parameters with the particular static one ...
rawTextFromStorage = strcat(rawTextFromStorage, rawTextFromStoragePart);
