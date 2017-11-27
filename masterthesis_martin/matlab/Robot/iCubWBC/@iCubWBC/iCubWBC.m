classdef iCubWBC < WBM.Interfaces.iCubWBM
    properties
        %% Structural Parameters
        model_name
        active_floating_base % switch to control if the icub has or not a floating base

        x_b                  % the cartesian position of the base (R^3)
        R_b                  % the rotation matrix describing the orientation of the base (global parametrization of SO(3))
        dx_b                 % the cartesian velocity of the base (R^3)
        omega_W              % the velocity describing the orientation of the base (SO(3))

        jointList            % the list of all the joints in the same order than the urdf
        linkList             % the list of all the links in the same order than the urdf
        revoluteJointList    % the list of all the not fixed joints in the same order than the urdf
        UBjointLimit         % Upper limit boundarie of all the revolute joints
        LBjointLimit         % Lower limit boundarie of all the revolute joints
        effortLimit          % Limit efforts of all the revolute joints
    end

    methods
        function obj = iCubWBC(robot_model, robot_config, wf2fixLnk)
            % call the constructor of the superclass ...
            obj = obj@WBM.Interfaces.iCubWBM(robot_model, robot_config, wf2fixLnk);
            [strPath, obj.model_name, ext] = fileparts(robot_model.urdf_robot_name);

            if strcmp(obj.model_name, 'icubGazeboSim')
                obj.active_floating_base = true;

            elseif strcmp(ext, '.urdf')
                obj.active_floating_base = false;

                if isempty(strPath)
                    urdf_file = which(robot_model.urdf_robot_name);
                else
                    urdf_file = robot_model.urdf_robot_name;
                end

                scheme = xml2struct(urdf_file);
                obj.UBjointLimit = [];
                obj.LBjointLimit = [];
                obj.effortLimit  = [];

                idx = 1;
                for i = 1:length(scheme.robot.link)
                    if( ~strcmp(scheme.robot.link{i}.Attributes.name, 'base_link') )
                        obj.linkList{idx} = scheme.robot.link{i}.Attributes.name;
                        idx = idx + 1;
                    end
                end

                idx = 1;
                for i = 1:length(scheme.robot.joint)
                    obj.jointList{i} = scheme.robot.joint{i};

                    if strcmp(obj.jointList{i}.Attributes.type, 'revolute')
                        obj.revoluteJointList{idx} = scheme.robot.joint{i};
                        obj.UBjointLimit = horzcat(obj.UBjointLimit, str2double(scheme.robot.joint{i}.limit.Attributes.upper));
                        obj.LBjointLimit = horzcat(obj.LBjointLimit, str2double(scheme.robot.joint{i}.limit.Attributes.lower));
                        obj.effortLimit  = horzcat(obj.effortLimit,  str2double(scheme.robot.joint{i}.limit.Attributes.effort));
                        idx = idx + 1;
                    end
                end
                obj.ndof = length(obj.revoluteJointList);
            else
                error('WBM::Interfaces::iCubWBC: Unknown robot model!');
            end
        end

        function SetWorldFrameiCub(obj, qjInit, dqjInit, dx_bInit, omega_WInit, reference_link)
            %% Updating the robot position
            setState(obj.mwbm_icub, qjInit, dqjInit, vertcat(dx_bInit, omega_WInit));
            % fixing the world reference frame w.r.t. the foot on ground position
            [x_b0, R_b0] = getWorldFrameFromFixLnk(obj.mwbm_icub, reference_link);
            % define world frame
            setWorldFrame(obj.mwbm_icub, R_b0, x_b0);

            % update position and orientation of the floating base repect of the root base
            obj.x_b = x_b0;
            obj.R_b = R_b0;
            % initial velocity floating base
            obj.dx_b    = dx_bInit;
            obj.omega_W = omega_WInit;
        end

        % wrapper-function for the robot's state:
        %
        % q_j    joint angles (NumDoF x 1)
        % vqT_b  base rototranslation (7 x 1) with 3 for position of frame and 4 for orientation quaternion
        %        (return is already flipped so that quaternion organised as real
        %        followed by imaginary)
        % dq_j   joint velocities (NumDoF x 1)
        % v_b    floating base velocity (6 x 1)
        function [q_j, vqT_b, dq_j, v_b] = GetState(obj)
            [vqT_b, q_j, v_b, dq_j] = getState(obj);
        end

        % wrapper-function for the whole body dynamics:
        %
        % M       mass matrix (inertia)
        % f       generalized bias forces
        % omega   centroidal momentum
        function [M, f, omega] = WholeBodyDynamics(obj, q_j, dq_j)
            [M, f, omega] = wholeBodyDyn(obj, q_j, dq_j);
        end

        % wrapper-function for the generalized bias forces:
        %
        % the hypothesis is that fc is already premultiplied by the Jc (contact jacobian)
        function f = F(obj, q_j, dq_j, f_c, Jc_t)
            f = genForces(obj, Jc_t, f_c, q_j, dq_j);
        end

        % wrapper-function for the mass-matrix:
        function M = inertia(obj, q_j)
            M = massMatrix(obj, q_j);
        end

        function suppConvHull = computeSupPoly(obj, feet_on_ground, chi)
            % Compute the support polygone w.r.t. feet_on_ground:
            if ( feet_on_ground(1) && feet_on_ground(2) )
                % if both feet on the ground the ref frame is l_sole
                % l_foot:
                [pos_lfoot,~] = offlineFkine(obj, chi, 'r_sole');

                X__lfoot = pos_lfoot(1);    Y_lfoot = pos_lfoot(2);
                X(1,1) = X__lfoot + 0.12;   Y(1,1) = Y_lfoot + 0.025;
                X(1,2) = X__lfoot + 0.12;   Y(1,2) = Y_lfoot - 0.025;
                X(1,3) = X__lfoot - 0.06;   Y(1,3) = Y_lfoot - 0.025;
                X(1,4) = X__lfoot - 0.06;   Y(1,4) = Y_lfoot + 0.025;

                % r_foot:
                [pos_rfoot,~] = obj.offlineFkine(chi, 'l_sole');

                X__rfoot = pos_rfoot(1);    Y_rfoot = pos_rfoot(2);
                X(1,5) = X__rfoot + 0.12;   Y(1,5) = Y_rfoot + 0.025;
                X(1,6) = X__rfoot + 0.12;   Y(1,6) = Y_rfoot - 0.025;
                X(1,7) = X__rfoot - 0.06;   Y(1,7) = Y_rfoot - 0.025;
                X(1,8) = X__rfoot - 0.06;   Y(1,8) = Y_rfoot + 0.025;
            end
            if ( ~feet_on_ground(1) || ~feet_on_ground(2) )
                % if only  left foot is on the ground the ref frame is l_sole
                % if only right foot is on the ground the ref frame is r_sole
                X(1,1) =  0.12;     Y(1,1) =  0.025;
                X(1,2) =  0.12;     Y(1,2) = -0.025;
                X(1,3) = -0.06;     Y(1,3) = -0.025;
                X(1,4) = -0.06;     Y(1,4) =  0.025;
            end
            suppConvHull = ConvexHull(X,Y);
        end

        % This function initialize and allow to use different limbs of the
        % robot according to the current URDF
        % The possible values of kin_chain are
        % 'trunk','left_arm','right_arm','l_sole','r_sole'
        % return qjInit in degres
        % If no initial joints values are passed through varargin the
        % function automatically set iCub to the default position
        function qjInit = InitializeState(obj, list_of_kin_chain, feet_on_ground, varargin)
            qjInit = zeros(obj.ndof,1);

            if ~isempty(varargin)
                joints_initial_values = varargin{1};

                if ~(length(joints_initial_values) == length(list_of_kin_chain))
                    error('joints_initial_values is malformed, should be the same lenght as list_of_kin_chain');
                end
            end

            if ~isempty( find(SubStrFind('trunk', list_of_kin_chain), 1) )
                if ~isempty(varargin)
                    torsoInit = joints_initial_values{1,find(SubStrFind('trunk', list_of_kin_chain), 1)};
                else
                    torsoInit = [0.0;  0.0;  0.0];
                end
                string_search = {'torso_yaw', 'torso_roll', 'torso_pitch'};
                qjInit = sortJointValue(obj, string_search, qjInit, torsoInit);
            end

            if ~isempty( find(SubStrFind('left_arm', list_of_kin_chain), 1) )
                if ~isempty(varargin)
                    leftArmInit = joints_initial_values{1,find(SubStrFind('left_arm', list_of_kin_chain), 1)};
                else
                    leftArmInit = [-20.0  30.0  0.0  45.0  0.0 0.0 0.0];
                end
                string_search = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', ...
                                 'l_elbow', 'l_wrist_prosup', 'l_wrist_pitch', 'l_wrist_yaw'};
                qjInit = sortJointValue(obj, string_search,qjInit,leftArmInit);
            end

            if(~isempty(find(SubStrFind('right_arm',list_of_kin_chain),1)))
                if (~isempty(varargin))
                    rightArmInit = joints_initial_values{1,find(SubStrFind('right_arm', list_of_kin_chain), 1)};
                else
                    rightArmInit = [-20.0  30.0  0.0  45.0  0.0 0.0 0.0];
                end
                string_search = {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', ...
                                 'r_elbow','r_wrist_prosup','r_wrist_pitch','r_wrist_yaw'};
                qjInit = sortJointValue(obj, string_search, qjInit, rightArmInit);
            end

            if ( ~isempty(find(SubStrFind('l_sole', list_of_kin_chain), 1)) || ...
                 ~isempty(find(SubStrFind('r_sole', list_of_kin_chain), 1)) )
                if ~isempty(varargin)
                    if ~isempty(find(SubStrFind('l_sole',list_of_kin_chain),1))
                        leftLegInit = joints_initial_values{1,find(SubStrFind('l_sole', list_of_kin_chain), 1)};
                    end
                    if ~isempty(find(SubStrFind('r_sole',list_of_kin_chain),1))
                        rightLegInit = joints_initial_values{1,find(SubStrFind('r_sole', list_of_kin_chain), 1)};
                    end
                else
                    if ( feet_on_ground(1) && feet_on_ground(2) )
                        % initial conditions for balancing on two feet
                        leftLegInit  = [25.5   0.1   0.0  -18.5  -5.5  -0.1];
                        rightLegInit = [25.5   0.1   0.0  -18.5  -5.5  -0.1];
                    elseif feet_on_ground(1)
                        % initial conditions for the robot standing on the left foot
                        leftLegInit  = [25.5   15.0   0.0  -18.5  -5.5  -0.1];
                        rightLegInit = [25.5   5.0    0.0  -40    -5.5  -0.1];
                    elseif feet_on_ground(2)
                        % initial conditions for the robot standing on the right foot
                        leftLegInit  = [25.5   5.0    0.0  -40    -5.5  -0.1];
                        rightLegInit = [25.5   15.0   0.0  -18.5  -5.5  -0.1];
                    end
                end
                string_search = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw','l_knee', ...
                                 'l_ankle_pitch', 'l_ankle_roll'};
                qjInit = sortJointValue(obj, string_search, qjInit, leftLegInit);

                string_search = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', ...
                                 'r_ankle_pitch', 'r_ankle_roll'};
                qjInit = sortJointValue(obj, string_search, qjInit, rightLegInit);
            end
            qjInit = qjInit.*(pi/180);
        end

        % Same as the method fkine but can be call at anytime. By tag you
        % specify through a string the name of the joint you want
        function [x, R] = offlineFkine(obj, chi, tag)
            n = obj.ndof;

            x_base = chi(1:3,1);  % TODO floating base flag required (parameter of the simulator)
            qt_b   = chi(4:7,1);  % TODO floating base flag required (parameter of the simulator)
            q      = chi(8:(n+7),1); % n+7 = 8+n-1

            vqT_b = vertcat(x_base, qt_b);
            [~,R_base] = WBM.utilities.tfms.frame2posRotm(vqT_b);

            fkine = forwardKinematics(obj.mwbm_icub, R_base, x_base, q, tag);
            % Obtaining the rotation matrix from root link to world frame
            [x, R] = WBM.utilities.tfms.frame2posRotm(fkine);
        end

        % Same as the method jacob0 but can be call at anytime. By tag you
        % specify through a string the name of the joint you want
        function jacob0 = offlineJacob0(obj, chi, tag)
            n = obj.ndof;

            x_base = chi(1:3,1);  % TODO floating base flag required (parameter of the simulator)
            qt_b   = chi(4:7,1);  % TODO floating base flag required (parameter of the simulator)
            q      = chi(8:(n+7),1); % n+7 = 8+n-1

            vqT_b = vertcat(x_base, qt_b);
            [~,R_base] = WBM.utilities.tfms.frame2posRotm(vqT_b);

            jacob0 = jacobian(obj.mwbm_icub, R_base, x_base, q, tag);
        end

        % Create the constraints_values vector need to compute the constraints
        % used in AllRUntimeParameters
        function vector = createConstraintsVector(obj)
            n = obj.ndof;
            vector = zeros(1,4*n);

            for i = 1:n
                vector(1,2*i-1) = obj.UBjointLimit(i);
                vector(1,2*i)   = obj.LBjointLimit(i);

                vector(1,2*(n+i)-1) =  obj.effortLimit(i); % 2*(n+i)-1 = 2*n+2*i-1
                vector(1,2*(n+i))   = -obj.effortLimit(i);
            end
        end

        function qjout = sortJointValue(obj, string_search, q_j, valueVector)
            %% sortJointValue
            % Sort the joints values valueVector of the joints listed in the cell of
            % strings string_search to match the same order in the joint position
            % vector qj as the order in the urdf file
            qjout = q_j;
            for i = 1:length(string_search)
                j = 1;
                while (j <= length(obj.revoluteJointList))
                    if strcmp(obj.revoluteJointList{j}.Attributes.name, string_search{i})
                        qjout(j) = valueVector(1,i);
                    end
                    j = j + 1;
                end
            end
        end

        [] = plot(obj, chi, params, varargin)

    end
end
