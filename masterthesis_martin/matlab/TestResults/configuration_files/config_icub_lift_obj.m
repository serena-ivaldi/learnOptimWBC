% Configuration script for the lift-object experiment:
%
% This configuration script creates and setup all necessary data types and
% parameters for the experiment simulation with the iCub humanoid robot of
% lifting an object (wooden cube) from a deposit table to a shelf.
%
% Note: This configuration file works only if the WBM-Library is installed
%       on the system.

%%%;;

import WBM.*
import WBM.RobotModel.iCub_arms_torso_free.*

simulator_type = {'icub_matlab'};

% Selected multi-task controller type ('GHC' or 'UF'):
CONTROLLERTYPE = 'UF';

% Time duration structure for the experiment:
time_struct.ti   = 0;     % init. time in [s]
time_struct.tf   = 20;    % final time in [s]
time_struct.step = 0.001; % time step in [s]

% Backup file for the current set parameters:
% name_dat = 'icub_atf_lift_obj_1.0';
% path = LoadParameters(name_dat);
% load(path);

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

%% Target points for the elementary tasks:
%
% Cartesian coordinates, elementary target points to be reached, for the left
% and right trajectory of each joint controlled by the system to achieve the
% desired learning behavior:
nTsk    = 11;
trg_pts = cell(1,nTsk);

% side length of the cube to be grabbed ...
l_s  = sim_config.environment.vb_objects(3,1).dimension(1,1);
ls_h = l_s * 0.5; % half side length

% High-level commands:
% (‡) ... elementary tasks
%
%   * GRAB_OBJECT_AT_POS([A,B])
%       + MOVE_HANDS_TO_POS(A,B)
%           - MOVE_R_HAND_TO_POS(A)  (‡)
%           - MOVE_L_HAND_TO_POS(B)  (‡)
%
%   * SET_JOINT_POSITIONS(jnt_pos)   (‡)
%
%   * MOVE_OBJECT_TO_POS([C,D])      % move obj. to interm. target pos.
%       + MOVE_HANDS_TO_POS(C,D)
%           - MOVE_R_HAND_TO_POS(C)  (‡)
%           - MOVE_L_HAND_TO_POS(D)  (‡)
%
%   * MOVE_OBJECT_TO_POS([E,F])      % move obj. to final target pos.
%       + MOVE_HANDS_TO_POS(E,F)
%           - MOVE_R_HAND_TO_POS(E)  (‡)
%           - MOVE_L_HAND_TO_POS(F)  (‡)
%
%   * RELEASE_OBJECT([G,H], [I,J])
%       + MOVE_HANDS_TO_POS(G,H)     % release obj.
%           - MOVE_R_HAND_TO_POS(G)  (‡)
%           - MOVE_L_HAND_TO_POS(H)  (‡)
%       + MOVE_HANDS_TO_POS(I,J)     % move hands back to end-pos.
%           - MOVE_R_HAND_TO_POS(I)  (‡)
%           - MOVE_L_HAND_TO_POS(J)  (‡)
%
% GRAB_OBJECT_AT_POS:
trg_pts{1,1}  = [0.20   ls_h  (0.50+ls_h)]; % move r_hand to targ. pos. A
trg_pts{1,2}  = [0.20  -ls_h  (0.50+ls_h)]; % move l_hand to targ. pos. B
% SET_JOINT_POSITIONS:
trg_pts{1,3}  = repmat(0.5, 1, ndof); % joint positions to minimize the torques in the fitness function.
% MOVE_OBJECT_TO_POS (interm. target points):
trg_pts{1,4}  = [0.20   ls_h  (0.70+ls_h)]; % move r_hand to interm. pos. C
trg_pts{1,5}  = [0.20  -ls_h  (0.70+ls_h)]; % move l_hand to interm. pos. D
% MOVE_OBJECT_TO_POS (final target points):
trg_pts{1,6}  = [0.35   ls_h  (0.70+ls_h)]; % move r_hand to targ. pos. E
trg_pts{1,7}  = [0.35  -ls_h  (0.70+ls_h)]; % move l_hand to targ. pos. F
% RELEASE_OBJECT:
% MOVE_HANDS_TO_POS (release obj.):
trg_pts{1,8}  = [0.10   (ls_h+0.05)  (0.70+ls_h)]; % move r_hand to pos. G
trg_pts{1,9}  = [0.10  -(ls_h+0.05)  (0.70+ls_h)]; % move l_hand to pos. H
% MOVE_HANDS_TO_POS (back to end-pos.):
trg_pts{1,10} = [0.01  -0.17  0.48]; % move r_hand to end-pos. I
trg_pts{1,11} = [0.01   0.04  0.48]; % move l_hand to end-pos. J

%% Reference parameters:

% Geometric parameter array for every trajectory type
% (functional or sampled) of an elementary task:
geom_parameters = trg_pts;

% Dimension selection array to specify which Cartesian dimension or
% joint of every elementary task should be controlled by the system:
%    - 1 ... dimension/joint active
%    - 0 ... dimension/joint not active
dim_of_task = cell(1,nTsk);
%        dimension:  x  y  z
dim_of_task(1,:) = {[1; 1; 1]};  % activated dimensions
dim_of_task{1,3} = ones(ndof,1); % activated joints

% Subchain array (array of links) with the end-effectors (ee) of a given
% kinematic chain tree which will be controlled by the system:
%
% Note: Each kinematic chain can have only one subchain array with at least one
%       element and the array must have at least one end-effector. Further links
%       or joint positions to be controlled by the system can be added to the
%       array in dependency of the given elementary tasks (one link for each
%       elementary task) of the specified learning problem.
%
% elem. task idx:  1         2        3        4         5         6         7         8         9         10        11
% target pos.   :  A         B                 C         D         E         F         G         H         I         J
subchain1   = {'r_hand', 'l_hand', 'none', 'r_hand', 'l_hand', 'r_hand', 'l_hand', 'r_hand', 'l_hand', 'r_hand', 'l_hand'}; % Cartesian space and/or joint space
target_link = {subchain1}; % add the array to the target links of the control system ...

% subchain1   = {'r_hand','r_elbow_1','l_hand','l_elbow_1','none','r_hand','l_hand'}; % space (cartesian or joint space)
% target_link = {subchain1};

% Parameters for the primary trajectories:
%
% Array of trajectory-controller types (one controller for each elementary task):
%    - cartesian     (UF)  ... Cartesian controller (for coordinates)
%    - impedance     (UF)  ... impedance controller
%    - cartesian_x   (GHC) ... Cartesian controller to control only the translation
%    - cartesian_rpy (GHC) ... Cartesian controller to control only the rotation in Euler angles (ZYX)
%    - joint    (GHC & UF) ... joint position controller
traj_type      = cell(1,nTsk);
traj_type(1,:) = {'cartesian'}; traj_type{1,3} = 'joint';

% Control type array (one type for each elementary task) to specify
% what should be controlled by the trajectory-controller:
%    - regulation (GHC) ... use controller to reach a point
%    - tracking   (GHC) ... use controller to follow a trajectory
%    - x           (UF) ... use Cartesian controller to control only the translation
%    - rpy         (UF) ... use Cartesian controller to control only the rotation in Euler angles (ZYX)
%    - none  (GHC & UF) ... if the controller has no parameters
control_type      = cell(1,nTsk);
control_type(1,:) = {'x'}; control_type{1,3} = 'none';

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
% END of Parameters for the primary trajectories.

% set for both hands the target positions to be reached:
%                   l_hand:       r_hand:
trg_pos = vertcat(trg_pts{1,2}, trg_pts{1,1}, ... % first targets (at object)
                  trg_pts{1,7}, trg_pts{1,6});    % final targets (goal position of obj.)

% create and setup the trajectory and target point objects for both
% hands and add them to the simulation configuration structure:
[lnk_traj, trg_pts] = setupTrajectories_liftObj(trg_pos);
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


% %%  REFERENCE PARAMETERS
% deg = pi/180;
% % primary trajectory
% traj_type      = {'cartesian','cartesian','cartesian','cartesian','joint','cartesian','cartesian'};
% control_type   = {'x','x','x','x','none','x','x'}; % rpy is also possible instead of x ... (none = control type has no parameters, x ... cartesian)
% type_of_traj   = {'sampled','sampled','sampled','sampled','sampled','sampled','sampled'}; % computing fixed time steps (func)
% geometric_path = {'fixed','fixed','fixed','fixed','fixed','fixed','fixed'}; % like the previous but for geometric
% time_law = {'none','none','none','none','none','none','none'}; % time restriction on every sub-chain (tstart, tfinish)

% %parameters first chains
% % cartesian coordinates for the trajectory (goal-points of each joint) - elementary tasks
% geom_parameters{1,1} = [0.3,-0.17,0.68];    %r_e_e_point (goal-point)
% geom_parameters{1,2} = [0.18,-0.23,0.68];   %[0.21,-0.25,0.68];	%r_elbow_point
% geom_parameters{1,3} = [0.3,0.0248,0.68];   %l_e_e_point
% geom_parameters{1,4} = [0.18,0.1038,0.68];  %[0.21,0.1138,0.68];   %l_elbow_point
% geom_parameters{1,5} = [0 0.785398163397448 0 0 0 0.523598775598299 0 0 0.785398163397448 0 0 0 0.523598775598299 0 0 0 0]; % 17 dof (joint-type), to minimize the torques in the fitness function
% geom_parameters{1,6} = [0.28,-0.16,0.7463];   %r_end_pt  [0.2512,-0.16,0.76] [0.28,0.0148,0.7463];
% geom_parameters{1,7} = [0.28,0.0148,0.7463];  %l_end_pt [0.21,0.0148,0.77]

% % dimension of every task (cartesian) ... activate or deactivate the dimensons (1,0,1)
% dim_of_task{1,1}=[1;1;1]; dim_of_task{1,2}= [1;1;1];
% dim_of_task{1,3}= [1;1;1]; dim_of_task{1,4}= [1;1;1];
% dim_of_task{1,5}= ones(bot1.ndof,1); % joint-type (maybe not working in the code)
% dim_of_task{1,6}= [1;1;1]; dim_of_task{1,7}= [1;1;1];


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
qi{1}  = []; % initial joint positions of the robot (empty = undefined)
qdi{1} = []; % initial joint velocities of the robot (empty = undefined)

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

% setup the initial body (joint) positions ...
params.qjInit = bot1.InitializeState(list_of_kin_chain, params.feet_on_ground, joints_initial_values);

% special initialization (direct coordinates in joint space) ...
% params.qjInit = [0.0606953245641302 1.41994832618389 -6.74178292930099e-08 -1.14179614686264e-07 -0.984861184692183 0.761783299026996 ...
%                  0.849107718577681 1.82446511832070e-07 1.23867031369701 -3.00036385696852e-08 6.49994328883225e-08 -1.14151145574591 ...
%                  0.575459848754039 0.945960370485754 -6.03357679751874e-08 0.324676495831434 -0.0358523080153591].';

params.dqjInit     = zeros(bot1.ndof,1); % initial angular joint velocities
params.dx_bInit    = zeros(3,1);         % initial linear base velocity (Cartesian velocity)
params.omega_bInit = zeros(3,1);         % initial angular base velocity

% Define the fixed reference link for the world frame (WF):
if params.feet_on_ground(1,1)
    params.root_reference_link = 'l_sole';
else
    params.root_reference_link = 'r_sole';
end

% the values of these parameters depending from the chosen controller type ...
params.tStart   = time_struct.ti;
params.tEnd     = time_struct.tf;
params.sim_step = 0.01; %time_struct.step;
%params.wait     = waitbar(0, 'State integration in progress...');
params.maxtime  = 100;

params.demo_movements    = 0;
params.torque_saturation = 100000;
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
        numeric_reference_parameter = {[0.047180 0.359539 1.045565 0.374223 -0.069047 0.013630 -0.495463 ...
                                        -0.131683 0.668327 -0.184017 1.115775 0.884010 0.120701 0.837400 1.189048].'};
        % Secondary reference parameters:
        secondary_numeric_reference_parameter = {[]};

        % Repeller parameters:
        %
        % Index vector to link each obstacle with a repeller object:
        % Note: If the index order of the given obstacles is changed, then the
        %       index order to the linked repeller objects must be also changed
        %       to the same order.
        rep_obstacle_ref = [1 2]; % 1 ... deposit table, 2 ... shelf
        % rep_obstacle_ref = [1 2 3]; % 1 ... deposit table, 2 ... shelf, 3 ... wooden cube (object to grab)

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
        % Note: To execute the result from the optimization step, the theta
        %       values in 'MainExec' must be changed to the optimized values.
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
            %        A  B     C  D  E  F  G  H  I  J
            theta = [8, 8, 1, 4, 4, 4, 4, 2, 2, 6, 6];

            j = 1;
            for i = 1:nTsk
                numeric_theta(1,j:(i*nb)) = v1*theta(1,i);
                j = j + nb;
            end

            % a = 14; b = 5; c = 14; d = 5; e = 1; f = 0; g = 0;
            % numeric_theta = [a a a a a b b b b b c c c c c d d d d d e e e e e f f f f f g g g g g];
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

        % metric = {'M^(1)','M^(1)','M^(1)','M^(1)','M^(1)','M^(1)','M^(1)'};

        % Primary task gains:
        kd = repmat(80, 1, nTsk); % one gain value for each task
        kp = repmat(30, 1, nTsk);

        % kd = [80,80,80,80,80,80,80];
        % kp = [30,30,30,30,30,30,30]; % row vector one for each chain

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
        %       The regularizer term is an array of vectors with chain regulation
        %       values. The size of each vector is the number of tasks of the
        %       current chain, i.e. one regulation value for each task.
        %
        % Primary chain:
        regularizer_chain_1 = zeros(1,nTsk);
        d = 0.001; % damp value
        regularizer_chain_1(1,1) = d; % task 1
        regularizer_chain_1(1,2) = d; % task 2
        regularizer_chain_1(1,6) = d; % task 6
        regularizer_chain_1(1,7) = d; % task 7

        % Secondary chain:
        regularized_chain_2 = 1;

        % regularizer_chain_1 = [0  0  0  0  0  0  0.001]; % primary chain
        % regularized_chain_2 = 1;                         % secondary chain

        regularizer = cell(1,2);
        regularizer{1,1} = regularizer_chain_1;
        regularizer{1,2} = regularized_chain_2;
        % END of Parameters for the UF-controller.

        % Constraint parameters:
        %
        constraints_values     = bot1.createConstraintsVector;
        distConstraints_values = ones(1,9) * 0.03; % distances for the collision constraints
        nCstrs  = length(constraints_values);
        nDCstrs = length(distConstraints_values);
        nCVals  = nCstrs + nDCstrs + 1; % total number of constraint values

        constraints_functions = cell(1,nCVals);

        % Constraint functions for the fitness function to compute
        % the constraint violations (angular position constraints):
        for k = 1:2:nCstrs
            constraints_functions{1,k}   = 'LinInequality';  % superior constraint
            constraints_functions{1,k+1} = 'LinInequality2'; % inferior constraint
        end

        % Constraint functions for the collision detection:
        for k = 1:nDCstrs
            constraints_functions{1,nCstrs+k} = 'DistanceObs'; % collision detection constraint
        end
        % add a penalty function to a fixed distance with a tolerance of epsilon ...
        constraints_functions{1,nCVals} = 'FixedDistanceEquality';

        l_cub = bot1.sim_config.environment.vb_objects(3,1).dimension(1,1);
        fixDistance = l_cub; % length of all sides of the wooden cube

        % Constraint values (one constant value for each constraint function):
        constraints_values = horzcat(constraints_values, distConstraints_values, fixDistance);

        % Constraint types:
        % Vector that specifies if the constraints are equality or inequality functions.
        %    - 1 ... inequality function
        %    - 0 ... equality function
        constraints_type = ones(1,nCstrs);
        constraints_type(1,nCstrs) = 0; % the last constraint is an equality function

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
        % fitness = @fitnessHumanoidsICub5;

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
        %           A  B     C  D  E  F  G  H  I  J
        theta_st = [5, 5, 6, 4, 4, 4, 4, 3, 3, 5, 5];

        j = 1;
        for i = 1:nTsk
            user_defined_start_action(1,j:(i*nb)) = v1*theta_st(1,i);
            j = j + nb;
        end

        % a = 5; b = 5; c = 5; d = 5; e = 6; % numeric theta starting points (not working need to be in this case 7 (number of tasks))
        % user_defined_start_action = [a a a a a b b b b b c c c c c d d d d d e e e e e];

        % Parameters for MainOptRobust:
        explorationRate = 0.1; %0.5; %[0 1];
        niter           = 100; % number of iterations (generations)
        % END of Parameters for MainOptRobust.

        % Set the value range (search space) for the CMA-ES algorithm:
        % Note: The value range will be used if the start points for
        %       the initial generation are set to 'random'.
        cmaes_value_range = [-14  14];

        % cmaes_value_range{1,1} = [-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-14,-0.15,-0.15,-0.15 ]; % lower bound that define the search space
        % cmaes_value_range{1,2} = [14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,0.15,0.15,0.15];                    % upper bound that define the search space

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
            %         A   B     C  D  E  F  G  H  I  J
            theta = [12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0];

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
        explorationRate = 0.1; %[0 1];
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
%rawTextFromStorage = strcat(rawTextFromStorage, rawTextFromStoragePart);
