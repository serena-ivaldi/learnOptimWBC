% stateMachineWalkingInPlace computes robot state and references of the WALKING_IN_PLACE demo.                   
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [state, references_CoM, references_LFoot, references_RFoot, references_rot_task, feetInContact, references_s, w_H_b] = stateMachineExample ...
%            (s_0, w_H_CoM_0, w_H_LFoot_0, w_H_RFoot_0, w_H_rot_task_0, t, LFoot_H_b, RFoot_H_b, LFoot_wrench, RFoot_wrench, Config)
%
% INPUT:  - s_0 = [ROBOT_DOF * 1] initial joint positions
%         - w_H_CoM_0 = [4 * 4] initial world to CoM transform
%         - w_H_LFoot_0 = [4 * 4] initial world to LFoot transform
%         - w_H_RFoot_0 = [4 * 4] initial world to RFoot transform
%         - w_H_rot_task_0 = [4 * 4] initial world to rotational task Link transform
%         - t = simulation time
%         - LFoot_H_b = [4 * 4] base to left foot transform
%         - RFoot_H_b = [4 * 4] base to right foot transform
%         - LFoot_wrench = [6 * 1] external forces and moments acting on the left foot
%         - RFoot_wrench = [6 * 1] external forces and moments acting on the right foot
%         - Config = user defined configuration
%
% OUTPUT: - state = current state of state machine
%         - references_CoM = [3 * 3] desired CoM position, velocity and acceleration
%         - references_LFoot = [6 * 5] desired LFoot pose, velocity
%                              and acceleration. NOTE that the format
%                              is: [pos, linear_vel, linear_acc, zeros(3,2);
%                                   Rotation, angular_vel, angular_acc] 
%         - references_RFoot = [6 * 5] desired RFoot pose, velocity and acceleration
%         - references_RotTask = [3 * 5] desired rotational task link orientation, angular velocity and acceleration.
%                                NOTE that the format is: [Rotation, angular_vel, angular_acc]     
%         - feetInContact = [2 * 1] feet in contact
%         - references_s = [ROBOT_DOF * 3] desired joint positions, velocities and accelerations
%         - w_H_b = [4 * 4] world to base transform
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [w_H_b, feetInContact, state, references_LFoot, references_RFoot, references_CoM, ...
          references_rot_task, references_LHand, references_RHand, references_s] = stateMachineWalkingInPlace ...
             (s_0, w_H_LFoot, w_H_LFoot_0, w_H_RFoot, w_H_RFoot_0, w_H_CoM, w_H_CoM_0, w_H_rot_task_0, ...
              w_H_LHand_0, w_H_RHand_0, LFoot_H_b, RFoot_H_b, LFoot_wrench, RFoot_wrench, t, Config, Sat)
          
     persistent currentState t_switch w_H_fixed_link stance_foot
     
     if isempty(currentState) || isempty(t_switch) || isempty(w_H_fixed_link)
         
         currentState = 1;
         t_switch = 0;
         w_H_fixed_link = eye(4);
         if Config.LFoot_in_contact_at0
            stance_foot = 0; %if left foot in contact, start balancing on left foot
         else
            stance_foot = 1; %if left foot not in contact, start balancing on right foot
         end
     end
     
     % STATE = 1: two feet balancing 
     %
     % Compute initial references for two feet balancing     
     references_LFoot    = [w_H_LFoot_0(1:3,4),      zeros(3,4);
                            w_H_LFoot_0(1:3,1:3),    zeros(3,2)];
     references_RFoot    = [w_H_RFoot_0(1:3,4),      zeros(3,4);
                            w_H_RFoot_0(1:3,1:3),    zeros(3,2)];
     references_CoM      = [w_H_CoM_0(1:3,4),        zeros(3,2)];
     references_rot_task = [w_H_rot_task_0(1:3,1:3), zeros(3,2)];
     references_s        = [s_0,                     zeros(size(s_0,1),2)];
     
     %Keep hands at their initial position with respect to rot_task frame
     rot_task_H_LHand_0 = transpose(w_H_rot_task_0) *  w_H_LHand_0;
     rot_task_H_RHand_0 = transpose(w_H_rot_task_0) *  w_H_RHand_0;
     %Desired position of the hand with respect to the world is obtained by
     %taking into account the rot_task frame position
     w_H_LHand_des = w_H_rot_task_0 * rot_task_H_LHand_0;
     w_H_RHand_des = w_H_rot_task_0 * rot_task_H_RHand_0;
     
     references_LHand    = [w_H_LHand_des(1:3,4),    zeros(3,4);
                            w_H_LHand_des(1:3, 1:3), zeros(3,2)];
     references_RHand    = [w_H_RHand_des(1:3,4),    zeros(3,4);
                            w_H_RHand_des(1:3, 1:3), zeros(3,2)];
    
     % Feet in contact
     feetInContact = [1,1];
    
     %Define stance and swing foot transformations
     if stance_foot == 0 %balancing on left foot
         stanceFoot_H_b    = LFoot_H_b;
         swingFoot_H_b     = RFoot_H_b;
         w_H_stanceFoot_0  = w_H_LFoot_0;
         swingFoot_wrench  = RFoot_wrench;
         
     else %balancing on right foot
         stanceFoot_H_b    = RFoot_H_b;
         swingFoot_H_b     = LFoot_H_b;
         w_H_stanceFoot_0  = w_H_RFoot_0;
         swingFoot_wrench  = LFoot_wrench;
     end
     
     
     % STATE = 1: initial balance on two feet
     %
     %currentState == 1
     % World to base transform (world frame coincides with the fixed link at 0)
     w_H_b = w_H_fixed_link*stanceFoot_H_b;
     
     % Change state
     if ~Config.ONLY_BALANCING && t > Config.t_balancing && currentState == 1
         currentState = 2;
         t_switch = t;
     end
     
              
     % STATE = 2: move CoM above the stance foot
     %
     if currentState == 2
         % World to base transform
         w_H_b = w_H_fixed_link*stanceFoot_H_b;
         
         % Keep CoM on stance foot
         references_CoM = [[w_H_stanceFoot_0(1:2,4);w_H_CoM_0(3,4)], zeros(3,2)];
         
         % Switch to the next state
         if norm(w_H_CoM(1:3,4) - references_CoM(:, 1)) < Config.precision_threshold && t > (t_switch + Config.t_balancing)
             
             currentState = 3; % stance foot balancing
             t_switch = t;
         end 
     end
        
     
     % STATE = 3: stance foot balancing
     %
     if currentState == 3
         
         % World to base transform
         w_H_b = w_H_fixed_link*stanceFoot_H_b;
         
         % Keep CoM on stance foot
         references_CoM = [[w_H_stanceFoot_0(1:2,4);w_H_CoM_0(3,4)], zeros(3,2)];
         
         % Feet in contact 
         % if stance_foot == 0, feetInContact = [1,0]; 
         % if stance_foot == 1, feetInContact = [0,1];
         feetInContact = [1-stance_foot, 1-~stance_foot];

         if stance_foot == 0 %left foot balancing
             % Move right foot backward and up
             references_RFoot    = [(w_H_RFoot_0(1:3,4)+transpose(Config.deltaPos_RFoot(currentState,:))), zeros(3,4);
                                     w_H_RFoot_0(1:3,1:3), zeros(3,2)];
         else %right foot balancing          
             % Move left foot backward and up
             references_LFoot    = [(w_H_LFoot_0(1:3,4)+transpose(Config.deltaPos_LFoot(currentState,:))), zeros(3,4);
                                     w_H_LFoot_0(1:3,1:3), zeros(3,2)];
         end
     
         % Switch to the next state
         if t > (t_switch + Config.t_balancing) 
            currentState = 4; % prepare for switching
            t_switch = t;
         end 
     end
     
     
     % STATE = 4: prepare for switching
     %
     if currentState == 4
         
         % World to base transform
         w_H_b = w_H_fixed_link*stanceFoot_H_b;
         
         % Keep CoM on stance foot
         references_CoM = [[w_H_stanceFoot_0(1:2,4); w_H_CoM_0(3,4)], zeros(3,2)];
        
         % Feet in contact 
         % if stance_foot == 0, feetInContact = [1,0]; 
         % if stance_foot == 1, feetInContact = [0,1];
         feetInContact = [1-stance_foot, 1-~stance_foot];
     
         % Switch to the next state
         if ((norm(w_H_RFoot(1:3,4) - w_H_RFoot_0(1:3, 4)) + norm(w_H_LFoot(1:3,4) - w_H_LFoot_0(1:3, 4))) < Config.precision_threshold) && (t > (t_switch + 1.2 * Config.t_balancing)) 
            currentState = 5; % back to two feet balancing
            t_switch = t;
         end 
     end
       
     
     % STATE = 5: back to two feet balancing
     %
     if currentState == 5
         
         % World to base transform
         w_H_b = w_H_fixed_link*stanceFoot_H_b;
         
         % Feet in contact
         if swingFoot_wrench(3) < Sat.toll_feetInContact
             % allow the swing foot to keep moving,
             % while it is being placed on the ground
             % if stance_foot == 0, feetInContact = [1,0];
             % if stance_foot == 1, feetInContact = [0,1];
             feetInContact = [1-stance_foot, 1-~stance_foot];
         end
         
         % Switch to the next state
         if norm(w_H_CoM(1:3,4) - references_CoM(:, 1)) < Config.precision_threshold && t > (t_switch + Config.t_balancing)        
             currentState = 2; % back to the beginning
             t_switch = t;
             %Switch swing foot
             stance_foot = double(~stance_foot);
             w_H_fixed_link = w_H_fixed_link*stanceFoot_H_b/swingFoot_H_b;
         end
     end
     
     % update state
     state = currentState;

end