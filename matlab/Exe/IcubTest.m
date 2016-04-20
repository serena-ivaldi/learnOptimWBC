clear all
close all
clc

icub = iCub();
chain_1 = DummyRvc_iCub(icub,'left_arm');


%SUBCHAIN PARAMETERS 
subchain1 = [5];
target_link{1} = subchain1;

robots{1} = chain_1;
chains = SubChains(target_link,robots);


%% simulator parameters

% balancing on two feet or one foot
feet_on_ground           =  [1,1];                                  %either 0 or 1; [left,right] (in the simulator)
% allows the visualization of torques, forces and other user-defined graphics 
visualizer_graphics      =  1;                                      %either 0 or 1
visualizer_demo          =  1;                                      %either 0 or 1
visualizer_jointsPos     =  0;                                      %either 0 or 1; only if visualizer_graphics = 1

% for the simulator            
leftArmInit  = [ -20   30  0.0  45   0.0]';          
rightArmInit = [ -20   30  0.0  45   0.0]'; 
torsoInit    = [ -10.0   0.0    0.0]';
if     obj.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
    % initial conditions for balancing on two feet 
     leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
     rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
elseif   obj.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
% initial conditions for the robot standing on the left foot
     leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
     rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 	 
elseif   obj.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
% initial conditions for the robot standing on the right foot
    leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  -0.1]';
    rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]'; 
end
qjInit      = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);
dqjInit     = zeros(obj.ndof,1);
dx_bInit    = zeros(3,1);
omega_bInit = zeros(3,1);

