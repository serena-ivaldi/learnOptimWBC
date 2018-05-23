ROBOT_DOF              = 23;
CONFIG.ON_GAZEBO       = true;

CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];

CONFIG.SMOOTH_DES_COM      = 0;    % If equal to one, the desired streamed values 
                                   % of the center of mass are smoothed internally 

CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                                   % of the postural tasks are smoothed internally 

%Robot configuration for WBT2.0
WBT_wbiList   = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';
WBT_robotName = 'icubSim';

if(strcmp(params.codyco,'old'))
    PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamicsTree/left_leg/cartesianEndEffectorWrench:o';
    PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamicsTree/right_leg/cartesianEndEffectorWrench:o';
    PORTS.RIGHT_ARM        = '/wholeBodyDynamicsTree/left_arm/endEffectorWrench:o';
    PORTS.LEFT_ARM         = '/wholeBodyDynamicsTree/right_arm/endEffectorWrench:o';
else
    PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
    PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
    PORTS.RIGHT_ARM        = '/wholeBodyDynamics/left_arm/endEffectorWrench:o';
    PORTS.LEFT_ARM         = '/wholeBodyDynamics/right_arm/endEffectorWrench:o';
end

PORTS.IMU              = ['/' WBT_robotName, '/inertial'];                                   
PORTS.HEAD             = ['/' WBT_robotName, '/head/state:o'];
dump.left_wrench_port  = ['/' WBT_robotName, '/left_foot/analog:o'];
dump.right_wrench_port = ['/' WBT_robotName, '/right_foot/analog:o'];

references.smoothingTimeMinJerkComDesQDes = 2.5;    %3;
sat.torque = 34;

CONFIG.smoothingTimeTranDynamics = 0.05;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
gain.qTildeMax         = 20*pi/180;
postures               = 0;  

gain.SmoothingTimeImp            = 1; 
gain.SmoothingTimeGainScheduling = 0.02;

%% PARAMETERS FOR TWO FEET ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    %% no backpack
    gain.PCOM                 = diag([10    10  10]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 0*sqrt(gain.PCOM);
    %% with backpack
%     gain.PCOM                 = diag([30    10  30]);
%     gain.ICOM                 = diag([  0    0   0]);
%     gain.DCOM                 = 0*sqrt(gain.PCOM);

    gain.PAngularMomentum     = 10;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 
%% no backpack
    impTorso            = [10   10   20
                            0    0    0];
                        
    impArms             = [10   10    10    8   
                            0    0     0    0  ];
                        
    impLeftLeg          = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 
%% backpack
%     impTorso            = [30   30   30
%                             0    0    0];
%                         
%     impArms             = [10   10    10    8   
%                             0    0     0    0  ];
%                         
%     impLeftLeg          = [ 30   30   30    60     10  10
%                              0    0    0     0      0   0]; 
% 
%     impRightLeg         = [ 30   30   30    60     10  10
%                              0    0    0     0      0   0]; 

                             
    intTorso            = [0   0    0];
    
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0     0  0    0  0];   
                                              
end

%% PARAMETERS FOR ONLY ONE FOOT ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
 
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);
    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 1 ;

    % Impedances acting in the null space of the desired contact forces 
    intTorso            = [0   0    0]; 
    
    intArms             = [0   0    0    0  ];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30                     % originally 20;
                            0    0    0]*scalingImp; 
    impArms             = [15   15    15    8   
                            0    0     0    0    ]*scalingImp;
                        
    impLeftLeg          = [ 30   30   30   120     10  10
                             0    0    0     0      0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]*scalingImp;                            
end

sat.integral              = 0;
gain.integral             = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances           = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = 0*sqrt(gain.impedances);
gain.increasingRatesImp   = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot
% phys.footSize              = [ -0.07 0.07  ;    % xMin, xMax
%                                -0.03 0.03 ];    % yMin, yMax    
                             
gain.footSize                = [ -0.07  0.12 ;    % xMin, xMax
                                 -0.045 0.05];    % yMin, yMax    
   
fZmin                        = 10;

gain.legSize                 = [ -0.025  0.025 ;   % xMin, xMax
                                 -0.005  0.005];   % yMin, yMax  

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol        = 1e-5;
reg.pinvDamp       = 0.01;
reg.pinvDampVb     = 1e-7;
reg.HessianQP      = 1e-5;
reg.impedances     = 0.1;
reg.dampings       = 0;
reg.norm_tolerance = 1e-4;
