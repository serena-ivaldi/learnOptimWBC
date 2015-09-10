classdef iCubWBM < handle
    properties(SetAccess = private)
        urdfName@char
        ndof@int16
        linkName@char
        R_lw
        p_lw
    end

    methods
        % Constructor:        
        function obj = iCubWBM(iCubParams)
            initRobot(iCubParams);
        end
        
        function initRobot(iCubParams)
            % Initialization:
            if nargin >= 1
                if ~exist('urdfName', 'var')
                    obj.urdfName = 'icubGazeboSim'; % default URDF
                else
                    obj.urdfName = urdfName;
                end
                obj.ndof = ndof;
                obj.linkName = linkName;
                obj.R_lw = R_lw;
                obj.p_lw = p_lw;
            else
                error('Wrong number of input arguments!');
            end
            
            % initialize the mex-wholeBodyModel of the iCub-Robot
            % by using the Unified Robot Description Format (URDF)
            % for the Gazebo simulator ...
            wbm_modelInitialize(obj.urdfName);
            
            % set the the world frame to a given rototranslation from
            % a chosen reference link ...
            wbm_setWorldLink(obj.linkName, obj.R_lw, obj.p_lw);            
        end
                
        function setState(q_j, dq_j, vw_b)
            % updates the robot state at every time step t, i.e. the joint positions,
            % joint velocities, the floating base velocity, etc.
            wbm_updateState(q_j, dq_j, vw_b);
        end
        
        function [q_j, xT_b, dq_j, v_b] = getState(varargin)
            [q_j, xT_b, dq_j, v_b] = wbm_getState();
        end
        
        function dispParameters(obj)
            sprintf(1, ['iCub-parameters:\n robot (URDF): %s\n' ' NDOF: %i\n' ...
                        'link name (frame): %s\n'], obj.urdfName, obj.ndof, obj.linkName);
        end

    end
end
