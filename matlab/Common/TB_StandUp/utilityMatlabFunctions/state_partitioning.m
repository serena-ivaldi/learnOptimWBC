function [ ss_position, ss_vel, robot_pos, robot_vel] = state_partitioning(x, robotDofs)
%STATE_PARTITIONING Summary of this function goes here
%   Detailed explanation goes here

seesaw_pos_len           = 4;
seesaw_vel_len           = 3;
robot_pos_len            = 7 + robotDofs;
robot_vel_len            = 6 + robotDofs;

seesaw_pos_initial_index = 1;
robot_pos_initial_index  = seesaw_pos_initial_index + seesaw_pos_len;
seesaw_vel_initial_index = robot_pos_initial_index  + robot_pos_len;
robot_vel_initial_index  = seesaw_vel_initial_index + seesaw_vel_len;

ss_position              = x(seesaw_pos_initial_index:seesaw_pos_initial_index + seesaw_pos_len - 1);
robot_pos                = x(robot_pos_initial_index : robot_pos_initial_index + robot_pos_len - 1);

ss_vel                   = x(seesaw_vel_initial_index : seesaw_vel_initial_index + seesaw_vel_len - 1);
robot_vel                = x(robot_vel_initial_index : robot_vel_initial_index + robot_vel_len - 1);

end
