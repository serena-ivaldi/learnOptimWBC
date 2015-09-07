classdef  iCubRobot < Controllers.AbstractController
    properties
        params;
    end

    methods
        % constructor:
        function obj = iCubRobot()
            % Initialization:
            %
            % initialize the mex-wholeBodyModel of the iCub-Robot
            % by using the Unified Robot Description Format (URDF)
            % for the Gazebo simulator ...
            wbm_modelInitialize('icubGazeboSim');

            %wbm_setWorldLink();
        end
        
        function obj = setState(q_j, dq_j, [dx_b; omega_W])
            % updates the robot state at every time step t, i.e. the joint positions, joint
            % velocities, the floating base velocity, etc.
            wbm_updateState(q_j, dq_j, [dx_b; omega_W]);
        end
        
        function [q_j, xT_b, dq_j, v_b] = getState()
           return [q_j, xT_b, dq_j, v_b] = wbm_getState();
        end


    end
end  
