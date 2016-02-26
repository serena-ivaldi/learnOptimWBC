classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        wf_R_b@double matrix
        wf_p_b@double vector
        wf_v_b@double vector
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
        base@double   matrix
    end

    properties(Access = protected)
        iwbm_robot@WBM.WBM
        istv@double    vector
        iwf_R_b@double matrix
        iwf_p_b@double vector
        iwf_v_b@double vector
        indof@uint16   scalar
        iconfig
    end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2FixLnk, L, varargin)
            if ( ~exist('robot_model', 'var') && ~exist('robot_config', 'var') )
                % base model parameters:
                robot_model = WBM.wbmBaseModelParams;
                robot_model.urdfRobot    = 'icubGazeboSim';
                robot_model.wf_R_rootLnk = eye(3,3);
                robot_model.wf_p_rootLnk = zeros(3,1);
                robot_model.g_wf         = [0; 0; -9.81];

                % base robot config:
                robot_config = WBM.wbmHumanoidConfig;
                robot_config.ndof          = 25;
                robot_config.nCstrs        = 2;
                robot_config.cstrLinkNames = {'l_sole', 'r_sole'};
                robot_config.dampCoeff     = 0.00; %0.75;
                % setup the body of the iCub-Robot with the initial body (joint) positions (in degrees):
                % note: this configuration assumes an iCub-Robot with 25 DoFs.
                robot_config.body           = WBM.Robot.iCub.setupBody_iCub();
                robot_config.jpos_torso     = [-10.0; 0.0; 0.0];
                robot_config.jpos_left_arm  = [-19.7; 29.7; 0.0; 44.9; 0.0];
                robot_config.jpos_left_leg  = [25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
                robot_config.jpos_right_arm = robot_config.pos_leftArm;
                robot_config.jpos_right_leg = robot_config.pos_leftLeg;
                % init-state parameters:
                robot_config.initStateParams.x_b     = zeros(3,1);
                robot_config.initStateParams.qt_b    = zeros(4,1);
                robot_config.initStateParams.q_j     = [robot_config.jpos_torso; robot_config.jpos_left_arm; robot_config.jpos_right_arm; ...
                                                        robot_config.jpos_left_leg; robot_config.jpos_right_leg] * (pi/180.0); % in radians
                robot_config.initStateParams.dx_b    = zeros(3,1);
                robot_config.initStateParams.omega_b = zeros(3,1);
                robot_config.initStateParams.dq_j    = zeros(robot_config.ndof,1);
            end
            
            if ~exist('wf2FixLnk', 'var') )
                wf2FixLnk = true;
            end

            % init the mex-WholeBodyModel for the iCub-Robot:
            iwbm_robot = WBM(robot_model, robot_config, wf2FixLnk);
        end

        function delete(obj)
            obj.delete();
        end

        function display(obj)

        end

        function actualizeState(obj)
            [wf_vqT_b, ~, obj.iwf_v_b, ~] = obj.iwbm_robot.getState();
            [obj.iwf_p_b, obj.iwf_R_b] = WBM.utilities.frame2posRotm(wf_vqT_b);
        end

        function T = A(obj, joints, q_j)

        end

        function T0_6 = T0_6(obj, q_j)

        end

        function J0 = jacob0(obj, q_j, varargin)            
            J0 = obj.iwbm_robot.jacobian(obj.iwf_R_b, obj.iwf_p_b, q_j);
        end

        function Jdot = jacob_dot(obj, q_j, dq_j)
            Jdot = obj.iwbm_robot.dJdq(obj.iwf_R_b, obj.iwf_p_b, q_j, dq_j, obj.iwf_v_b);
        end

        function M = inertia(obj, robot, q_j)
            M = obj.iwbm_robot.massMatrix(obj.iwf_R_b, obj.iwf_p_b, q_j);
        end

        function C = coriolis(obj, robot, q_j, dq_j)
            C = obj.iwbm_robot.coriolisCentrifugalForces(obj.iwf_R_b, obj.iwf_p_b, q_j, dq_j, obj.iwf_v_b);
        end

        function tg = gravload(obj, robot, q_j, grav)
            tg = obj.iwbm_robot.gravityForces(obj.iwf_R_b, obj.iwf_p_b, q_j);
        end

        function set.base(obj, v)
            if isempty(v)
                obj.base = eye(4,4);
            elseif ~ishomog(v)
                error('iCubWBM::set.base: The base must be a homogeneous transform!');
            else
                obj.base = v;
            end
        end

        function payload(obj, m, p)

        end

        function wf_R_b = get.wf_R_b(obj)

        end

        function wf_p_b = get.wf_p_b(obj)

        end

        function wf_v_b = get.wf_v_b(obj)

        end

        function robot_model = get.robot_model(obj)

        end

        function robot_config = get.robot_config(obj)

        end

    end
end
