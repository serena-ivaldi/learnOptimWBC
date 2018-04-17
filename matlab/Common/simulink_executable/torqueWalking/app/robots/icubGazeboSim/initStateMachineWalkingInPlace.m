% INITSTATEMACHINEEXAMPLE initializes the robot configuration for running
%                        'EXAMPLE_STATEMACHINE' demo. 
%
% USAGE: please note that this function is automatically executed when
%        running the Simulink model.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---

% SIMULATION SETUP
%
% Frames name list
Frames.IMU = 'imu_frame';
Frames.COM = 'com';
Frames.BASE_LINK = 'root_link';
Frames.LEFT_FOOT = 'l_sole';
Frames.RIGHT_FOOT = 'r_sole';
Frames.ROT_TASK_LINK = 'neck_2';
Frames.LEFT_HAND = 'l_wrist_1';
Frames.RIGHT_HAND = 'r_wrist_1';

% If true, joint references are modified in order not to be in conflict with
% the Cartesian tasks. The new joint references are calculated by means of an
% integration based inverse kinematics
Config.USE_INVERSE_KINEMATICS = true;

% If true, the QP problem will be defined with strict task priorities; if false,
% with soft task priorities between tasks and posture
Config.QP_USE_STRICT_TASK_PRIORITIES = false;

% If true, the output of QP solver will be forced to be continuous
Config.QP_USE_CONTINUITY_CONSTRAINTS = false;
Config.QP_IKIN_USE_CONTINUITY_CONSTRAINTS = false;

% If true, the IMU orientation is used in order to estimate the
% base-to-world transformation matrix
Config.USE_IMU4EST_BASE = false;

% If true, the orientation provided by the IMU is corrected using the neck
% positions (requires the neck port to be active)
Config.CORRECT_IMU_WITH_NECK_POS = true;

% If true, IMU pitch and yaw are not considered for estimating base-to-world transform
Config.FILTER_IMU_YAW = false;
Config.FILTER_IMU_PITCH = false;

% True if left foot is initially in contact with the ground (if false,
% right foot is assumed to be in contact) (EXAMPLE_STATEMACHINE DEMO ONLY)
Config.LFoot_in_contact_at0 = true; %false;

% If true, the robot will just balance on two feet (EXAMPLE_STATEMACHINE DEMO ONLY)
Config.ONLY_BALANCING = false;

% If Config.ONLY_BALANCING = false, this is the time the robot will balance
% before it starts moving (EXAMPLE_STATEMACHINE DEMO ONLY)
Config.t_balancing = 2;

% If true, simulation is stopped when qpOASES outputs a "-2" error (QP is unfeasible)
Config.CHECK_QP_ERROR = true;

% If true, simulation is stopped when the CoM of the robot goes below half of its initial height
Config.CHECK_ROBOT_FALLING = true;

%% Robot setup 

% Joint torque saturation
Sat.tau_max = 60; % [Nm]

% Saturation on torque derivative (for QP solver)
Sat.tauDot_max = 10000;

% Saturation on state jerk (for QP based inverse kinematics)
Sat.nuDDot_max = 10000;


% Weight matrix for the cartesian tasks
%% For learnOptimWBC
%weightRotTask; weightStanceFoot; weightSwingFoot; weightPostural; weight_tau; were obtained from inputData.mat 
Sat.weightCoM        = 1; %weightCoM is not included since it is set to be a constant value of 1
Sat.weightRotTask    = weightRotTask;
Sat.weightStanceFoot = weightStanceFoot;
Sat.weightSwingFoot  = weightSwingFoot;
Sat.weightLeftHand   = 0; %put to 0 if you don't want to let hands move freely
Sat.weightRightHand  = 0; %put to 0 if you don't want to let hands move freely
Sat.weightTasks      = zeros(3 + 3 + 6*4); %Matrix of all the weights, of the shape
% Sat.weightTasks = diag([ones(3,1) * 1; %weightCoM, constant;
%                         ones(3,1) * weightRotTask; %weights(1); %
%                         ones(6,1) * weightStanceFoot; %weights(2); %
%                         ones(6,1) * weightSwingFoot; %weights(3); %
%                         ones(6,1) * Sat.weightLeftHand; %constant, 0
%                         ones(6,1) * Sat.weightRightHand]); %constant, 0

% Weight for the postural minimization task
Sat.weightPostural = weightPostural; %0.001; weights(4); %

% Weight for the joint minimization task
Sat.weight_tau = weight_tau; %0.1*Sat.weightPostural; weights(5); %

% Numerical tolerance for assuming a foot on contact
Sat.toll_feetInContact = 0.1;

% Damping for the pseudoinverse used for computing the floating base velocity
Sat.pinvDamp_nu_b = 1e-6;

% If true, the feet accelerations are zero when the foot is in contact. If false, 
% feet accelerations are equal to a feedforward + feedback terms
Sat.zeroAccWhenFeetInContact = false;

%% Parameters for motors reflected inertia

% inverse of the transmission ratio
Config.invGamma = 100*eye(ROBOT_DOF);
% torso yaw has a bigger reduction ratio
Config.invGamma(3,3) = 200;

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff     = 0;

%% Smoothing of reference trajectories

% If true, reference trajectories are smoothed internally
Config.SMOOTH_COM_REF      = true;
Config.SMOOTH_LFOOT_POS    = true;
Config.SMOOTH_RFOOT_POS    = true;
Config.SMOOTH_LFOOT_ORIENT = true; 
Config.SMOOTH_RFOOT_ORIENT = true; 
Config.SMOOTH_ROT_TASK_REF = true;
Config.SMOOTH_JOINT_REF    = true; 
Config.SMOOTH_LHAND_POS    = true;
Config.SMOOTH_RHAND_POS    = true;
Config.SMOOTH_LHAND_ORIENT = true; 
Config.SMOOTH_RHAND_ORIENT = true;

% Smoothing time for tasks and joints references [s]
Config.smoothingTime_CoM    = 2*[1;1;1;1;1];
Config.smoothingTime_LFoot  = 2*[1;1;1;1;1];
Config.smoothingTime_RFoot  = 2*[1;1;1;1;1];
Config.smoothingTime_joints = 2*[1;1;1;1;1];
Config.smoothingTime_LHand  = 2*[1;1;1;1;1];
Config.smoothingTime_RHand  = 2*[1;1;1;1;1];

% Gains that will influence the smoothing of reference orientations. The
% higher, the faster. Only positive or null values.
Config.LFoot_Kp_smoothing    = 2*[1;1;1;1;1];
Config.LFoot_Kd_smoothing    = 2*[1;1;1;1;1];
Config.RFoot_Kp_smoothing    = 2*[1;1;1;1;1];
Config.RFoot_Kd_smoothing    = 2*[1;1;1;1;1];
Config.rot_task_Kp_smoothing = 2*[1;1;1;1;1];
Config.rot_task_Kd_smoothing = 2*[1;1;1;1;1];
Config.LHand_Kp_smoothing    = 2*[1;1;1;1;1];
Config.LHand_Kd_smoothing    = 2*[1;1;1;1;1];
Config.RHand_Kp_smoothing    = 2*[1;1;1;1;1];
Config.RHand_Kd_smoothing    = 2*[1;1;1;1;1];

% Smoothing time for gain scheduling [s].
Config.smoothingTimeGains    = 2*[1;1;1;1;1];

%% CoM and feet references

% add a delta to the right foot position. 
%
% dimension: [m]
% format: [x;y;z]
%
Config.deltaPos_RFoot = [ 0.000 0.00  0.000; ...   % state = 1 two feet balancing
                          0.000 0.00  0.000; ...   % state = 2 move CoM on left foot
                         -0.025 0.00  0.025; ...   % state = 3 left foot balancing
                          0.000 0.00  0.000; ...   % state = 4 prepare for switching
                          0.000 0.00  0.000];      % state = 5 two feet balancing
                      
Config.deltaPos_LFoot = [ 0.000 0.00  0.000; ...   % state = 1 two feet balancing
                          0.000 0.00  0.000; ...   % state = 2 move CoM on right foot
                         -0.025 0.00  0.025; ...   % state = 3 right foot balancing
                          0.000 0.00  0.000; ...   % state = 4 prepare for switching
                          0.000 0.00  0.000];      % state = 5 two feet balancing                      
   
%% Gains matrices

% CoM position and velocity gains
Gains.Kp_CoM = [50, 50, 50; ...  % state = 1 two feet balancing
                50, 50, 50; ...  % state = 2 move CoM on left foot
                50, 50, 50; ...  % state = 3 left foot balancing
                50, 50, 50; ...  % state = 4 prepare for switching
                50, 50, 50];     % state = 5 two feet balancing
                
Gains.Kd_CoM = 2*sqrt(Gains.Kp_CoM);

% Feet position and velocity gains
Gains.Kp_LFoot = [50, 50, 50, 30, 30, 30; ... % state = 1 two feet balancing
                  50, 50, 50, 30, 30, 30; ... % state = 2 move CoM on left foot
                  50, 50, 50, 30, 30, 30; ... % state = 3 left foot balancing
                  50, 50, 50, 30, 30, 30; ... % state = 4 prepare for switching
                  50, 50, 50, 30, 30, 30];    % state = 5 two feet balancing
              
Gains.Kd_LFoot = 2*sqrt(Gains.Kp_LFoot);

Gains.Kp_RFoot = [50, 50, 50, 30, 30, 30; ... % state = 1 two feet balancing
                  50, 50, 50, 30, 30, 30; ... % state = 2 move CoM on left foot
                  50, 50, 50, 30, 30, 30; ... % state = 3 left foot balancing
                  50, 50, 50, 30, 30, 30; ... % state = 4 prepare for switching
                  50, 50, 50, 30, 30, 30];    % state = 5 two feet balancing

Gains.Kd_RFoot = 2*sqrt(Gains.Kp_RFoot); 

% Root link orientation and angular velocity gains
Gains.Kp_rot_task = [20, 20, 20; ...  % state = 1 two feet balancing
                     20, 20, 20; ...  % state = 2 move CoM on left foot
                     20, 20, 20; ...  % state = 3 left foot balancing
                     20, 20, 20; ...  % state = 4 prepare for switching
                     20, 20, 20];     % state = 5 two feet balancing
                 
Gains.Kd_rot_task =  2*sqrt(Gains.Kp_rot_task); 

% Hand position and velocity gains
Gains.Kp_LHand = 0*[50, 50, 50, 30, 30, 30; ... % state = 1 two feet balancing
                  50, 50, 50, 30, 30, 30; ... % state = 2 move CoM on left foot
                  50, 50, 50, 30, 30, 30; ... % state = 3 left foot balancing
                  50, 50, 50, 30, 30, 30; ... % state = 4 prepare for switching
                  50, 50, 50, 30, 30, 30];    % state = 5 two feet balancing
              
Gains.Kd_LHand = 2*sqrt(Gains.Kp_LHand);

Gains.Kp_RHand = 0*[50, 50, 50, 30, 30, 30; ... % state = 1 two feet balancing
                  50, 50, 50, 30, 30, 30; ... % state = 2 move CoM on left foot
                  50, 50, 50, 30, 30, 30; ... % state = 3 left foot balancing
                  50, 50, 50, 30, 30, 30; ... % state = 4 prepare for switching
                  50, 50, 50, 30, 30, 30];    % state = 5 two feet balancing

Gains.Kd_RHand = 2*sqrt(Gains.Kp_RHand); 

% Joint position and velocity gains    
                    % torso      % left arm       % right arm      % left leg               % right leg                                   
Gains.impedances = [20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10;  ... % state = 1 two feet balancing          
                    20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10;  ... % state = 2 left foot balancing
                    20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10;  ... % state = 3 left foot balancing
                    20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10;  ... % state = 4 prepare for switching
                    20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10]; ... % state = 5 two feet balancing
                
Gains.dampings   = 0 * sqrt(Gains.impedances); %zeros(size(Gains.impedances));

% Joints position and velocity gains for inverse kinematics
Gains.ikin_impedances = Gains.impedances(1,:);
Gains.ikin_dampings   = 2*sqrt(Gains.ikin_impedances); 
    
%% Constraints for QP for balancing - friction cone - z-moment - in terms of f

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the 
% circle in each cicle's quadrant 
numberOfPoints               = 4; 
forceFrictionCoefficient     = 1;  
torsionalFrictionCoefficient = 1/75;
fZmin                        = 10; % Min vertical force [N]

% Size of the foot
Config.footSize              = params.footSize;  %[-0.05  0.10;     % xMin, xMax
                                                 % -0.025 0.025];   % yMin, yMax 
                            