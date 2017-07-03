%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%clear; clc;
%% GENERAL SIMULATION INFO
% If you are simulating the robot with Gazebo, 
% remember that you have to launch Gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
% 
% and set the environmental variable YARP_ROBOT_NAME = icubGazeboSim.
% To do this, you can uncomment the 


%% for stand alone execution remove comment on this line
%load('inputData.mat')


% setenv('YARP_ROBOT_NAME','iCubGenova01');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
%setenv('YARP_ROBOT_NAME','iCubGenova04');
% setenv('YARP_ROBOT_NAME','iCubDarmstadt01');
 setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova05');

% Simulation time in seconds
CONFIG.SIMULATION_TIME = params.tEnd;   

%% PRELIMINARY CONFIGURATIONS 
% SM.SM_TYPE: defines the kind of state machines that can be chosen.
%
% 'YOGA': the robot will perform the YOGA++ demo. The associated
%         configuration parameters can be found under the folder
%
%         robots/YARP_ROBOT_NAME/initStateMachine.m
%   
% 'COORDINATOR': the robot can either stay still, or follow a
%                center-of-mass trajectory, or follow references for the
%                joints. The associated configuration parameters can be 
%                found under the folder
%
%                robots/YARP_ROBOT_NAME/initRegGen.m
%
% 'STANDUP': the robot will stand up from a chair. The associated
%            configuration parameters can be found under the folder
%
%            robots/YARP_ROBOT_NAME/initStateMachineStandUp.m
%
% 'WALKING': under development.
%
SM.SM_TYPE                    = 'COORDINATOR';

% CONFIG.SCOPES: if set to true, all visualizers for debugging are active
CONFIG.SCOPES.ALL             = true;

% You can also activate only some specific debugging scopes
CONFIG.SCOPES.BASE_EST_IMU    = false;
CONFIG.SCOPES.EXTWRENCHES     = false;
CONFIG.SCOPES.GAIN_SCHE_INFO  = false;
CONFIG.SCOPES.MAIN            = false;
CONFIG.SCOPES.QP              = false;

% CONFIG.CHECK_LIMITS: if set to true, the controller will stop as soon as 
% any of the joint limit is touched. 
CONFIG.CHECK_LIMITS           = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, 
WBT_modelName                 = 'matlabTorqueBalancing';

% CONFIG.USE_IMU4EST_BASE: if set to false, the base frame is estimated by 
% assuming that either the left or the right foot stay stuck on the ground. 
% Which foot the  controller uses depends on the contact forces acting on it. 
% If set to true, the base orientation is estimated by using the IMU, while
% the base position by assuming that the origin of either the right or the
% left foot do not move. 
CONFIG.USE_IMU4EST_BASE      = false;
 
% CONFIG.YAW_IMU_FILTER and CONFIG.PITCH_IMU_FILTER: when the flag
% CONFIG.USE_IMU4EST_BASE = true, then the orientation of the floating base is
% estimated as explained above. However, the foot is usually perpendicular
% to gravity when the robot stands on flat surfaces, and rotation about the
% gravity axis may be de to the IMU drift in estimating this angle. Hence,
% when either of the flags CONFIG.YAW_IMU_FILTER or CONFIG.PITCH_IMU_FILTER
% is set to true, then the yaw and/or pitch angles of the contact foot are
% ignored and kept equal to the initial values.
CONFIG.YAW_IMU_FILTER        = true;
CONFIG.PITCH_IMU_FILTER      = true;

% CONFIG.CORRECT_NECK_IMU: when set euqal to true, the kineamtics from the
% IMU and the contact foot is corrected by using the neck angles. If it set
% equal to false, recall that the neck is assumed to be in (0,0,0)
CONFIG.CORRECT_NECK_IMU      = true;

% CONFIG.ONSOFTCARPET: the third year CoDyCo review meeting consisted also
% of a validation scenarion in which the robot had to balance on a soft
% carpet. Hence, when CONFIG.ONSOFTCARPET = true, other sets of gains are
% loaded for the postural and CoM.
CONFIG.ONSOFTCARPET          = false;

% CONFIG.USE_QP_SOLVER: if set to true, a QP solver is used to account for 
% inequality constraints of contact wrenches
CONFIG.USE_QP_SOLVER         = true; 

% YARP PORTS
PORTS.IMU                = '/icub/inertial';
PORTS.COM_DES            = ['/' WBT_modelName '/comDes:i'];
PORTS.Q_DES              = ['/' WBT_modelName '/qDes:i'];
PORTS.WBDT_LEFTLEG_EE    = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE   = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
PORTS.RIGHT_ARM          = '/wholeBodyDynamics/left_arm/endEffectorWrench:o';
PORTS.LEFT_ARM           = '/wholeBodyDynamics/right_arm/endEffectorWrench:o';

CONFIG.Ts                = 0.01;  % Controller period [s]

CONFIG.ON_GAZEBO         = false;
baseToWorldRotationPort  = ['/' WBT_modelName '/floatingBaseRotationMatrix:i'];

run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gains.m')); 
addpath('./src/')
addpath('./utilityMatlabFunctions/')

robotSpecificReferences = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initRefGen.m');
run(robotSpecificReferences);

SM.SM.MASK.COORDINATOR   = bin2dec('001');
SM.SM.MASK.YOGA          = bin2dec('010');
SM.SM.MASK.WALKING       = bin2dec('100');
SM.SM.MASK.STANDUP       = bin2dec('110');

SM.SM_TYPE_BIN           = SM.SM.MASK.COORDINATOR;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% Run all the initialization files for finite state machines. The reason is 
% there are variables used only in each initialization file: in this way,
% all variables required by the Simulink model will be initialized. Don't 
% worry, all common variables will be overwritten by calling again the 
% function specific for your demo. It is not really clean, but this is how
% things work for now.
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
robotSpecificFSM         = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachine.m');
run(robotSpecificFSM);
robotSpecificFSM         = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachineStandUp.m');
run(robotSpecificFSM);

%% iCub stand up demo parameters
% whwn both CONFIG.iCubStandUp and CONFIG.useExtHandForces are setted to TRUE,
% the robot will be aware of the external forces at the arms and will use
% also them for lifting up.
CONFIG.iCubStandUp       = true;
CONFIG.useExtArmForces   = false;

%% Define which simulation will be performed
if strcmpi(SM.SM_TYPE, 'COORDINATOR')
    SM.SM_TYPE_BIN          = SM.SM.MASK.COORDINATOR;
elseif strcmpi(SM.SM_TYPE, 'YOGA')
    SM.SM_TYPE_BIN          = SM.SM.MASK.YOGA;
    robotSpecificFSM        = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachine.m');
    run(robotSpecificFSM);
elseif strcmpi(SM.SM_TYPE, 'WALKING')
    SM.SM_TYPE_BIN          = SM.SM.MASK.WALKING;
    robotSpecificFSM        = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachineWalking.m');
    run(robotSpecificFSM);
elseif strcmpi(SM.SM_TYPE, 'STANDUP')
    SM.SM_TYPE_BIN          = SM.SM.MASK.STANDUP;
    CONFIG.iCubStandUp      = true;
    robotSpecificFSM        = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachineStandUp.m');
    run(robotSpecificFSM);
end
gain.impedances
%% Contact constraints: legs and feet
% feet constraints
[ConstraintsMatrixFeet,bVectorConstraintsFeet] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);
% legs constraints
[ConstraintsMatrixLegs,bVectorConstraintsLegs] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.legSize,fZmin);
