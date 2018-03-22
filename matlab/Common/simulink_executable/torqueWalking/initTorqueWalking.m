% INITTORQUEWALKING initializes the robot user defined configuration, gains and
%                      regularization parameters for the Simulink balancing controller.
%
% USAGE: please note that this function is automatically executed when
%        running or compiling the Simulink model.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---

%% for stand alone execution remove comment on the following lines
% clc
% clear all  %#ok<CLALL>
% close all
% load('inputData.mat')
%

% NOTE: if you are simulating the robot with Gazebo, remember that you have
% to launch Gazebo as follow:
% 
%     gazebo -slibgazebo_yarp_clock.so
%
% Set the YARP_ROBOT_NAME environmental variable
setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','iCubGenova04');


% SELECT THE DEMO TO BE PERFORMED:
%
%    - 'EXAMPLE_STATEMACHINE' = the robot balances and performs simple movements using 
%                               a state machine.
%
DEMO_TYPE = 'WALKING_IN_PLACE';

% Simulation time
Config.t_end = params.tEnd; %20; %inf; % [s]

% VISUALIZATION SETUP
%
% Activate all scopes in the model for visualization and debug
Config.SCOPES_ALL = true;

% Activate scopes related to forces and torques visualization
Config.SCOPES_TORQUES_AND_FORCES = false;

% Activate scopes for visualizing inverse kinematics results
Config.SCOPES_INVERSE_KINEMATICS = false;

% Activate scopes for visualizing forward kinematics results
Config.SCOPES_FORWARD_KINEMATICS = false;

% Activate scopes for visualizing the robot state
Config.SCOPES_ROBOT_STATE = false;

% Activate scopes for visualizing the smoothed reference orientations
Config.SCOPES_SMOOTH_ORIENT = false;

% Activate scopes for visualizing the control gains
Config.SCOPES_GAIN_SCHEDULING = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Advanced setup - do not change these parameters unless you know what you're doing
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add path to the src folder and subfolders
addpath(genpath('./src'));
addpath(genpath('./library'));

% Simulation step (fixed step integrator)
Config.t_step = 0.01; % [s]

%% STARTUP PROCEDURE
%
% A file called configRobot.m which contains a list of robot specific
% parameters, but in common for all demos is run. Then, the configuration
% file corresponding to the specified demo is run.
%
configRobotFCN = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'configRobot.m');
run(configRobotFCN);

if strcmp(DEMO_TYPE, 'WALKING_IN_PLACE')
    
    % Run configuration script for internal coordinator
    stateMachineWalkingInPlaceFCN = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'initStateMachineWalkingInPlace.m');
    run(stateMachineWalkingInPlaceFCN);
end

% Compute the constraint matrix and bias vector for friction and unilateral
% constraints at contact locations
[ConstraintMatrix_feet, biasVectorConstraint_feet] = constraints ...
    (forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, Config.footSize, fZmin);

disp('Initialize torqueWalking')
disp(['Selected robot: ',getenv('YARP_ROBOT_NAME')])
disp(['Selected demo: ', DEMO_TYPE])