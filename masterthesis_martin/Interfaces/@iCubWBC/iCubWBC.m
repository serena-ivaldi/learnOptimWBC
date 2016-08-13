classdef iCubWBC < WBM.Interfaces.iCubWBM
    properties
        %% Structural Parameters
        model_name
        active_floating_base % switch to control if the icub has or not a floating base

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
            [~,obj.model_name,~] = fileparts(robot_model.urdf_link_name);

            if strcmp(obj.model_name, 'icubGazeboSim')
                obj.active_floating_base = true;
                %obj.ndof = 25; % degrees of freedom without floating base

            elseif ( strcmp(obj.model_name, 'model_arms_torso_free') || strcmp(obj.model_name, 'model32dof') )
                obj.active_floating_base = false;

                %model   = strcat(obj.model_name, '.urdf');
                %strPath = which(model);
                strPath = which(robot_model.urdf_link_name);

                scheme = xml2struct(strPath);
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
                error('WBM::Interfaces::iCubWBC: Unknown URDF-model!');
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

        % the hypothesis is that fc is already premultiplied by the Jc (contact jacobian)
        function f = F(obj, q_j, dq_j, f_c, Jc_t)
            %f = wbm_generalisedBiasForces(obj.R_b, obj.x_b, q_j, dq_j, [obj.dx_b; obj.omega_W]) - Jc_t*fc;
            f = generalizedForces(obj, q_j, dq_j, f_c, Jc_t);
        end

        function suppConvHull = computeSupPoly(obj, feet_on_ground, chi)
        %Compute the support polygone wrt feet_on_ground

            if ( (feet_on_ground(1) == 1) && (feet_on_ground(2) == 1) )
                %if both feet on the ground the ref frame is l_sole
                %l_foot
                [pos_lfoot,~] = obj.offlineFkine(chi, 'r_sole');

                X__lfoot = pos_lfoot(1);    Y_lfoot = pos_lfoot(2);
                X(1,1) = X__lfoot + 0.12;   Y(1,1) = Y_lfoot + 0.025;
                X(1,2) = X__lfoot + 0.12;   Y(1,2) = Y_lfoot -0.025;
                X(1,3) = X__lfoot - 0.06;   Y(1,3) = Y_lfoot - 0.025;
                X(1,4) = X__lfoot - 0.06;   Y(1,4) = Y_lfoot + 0.025;

                %r_foot
                [pos_rfoot,~] = obj.offlineFkine(chi, 'l_sole');

                X__rfoot = pos_rfoot(1);    Y_rfoot = pos_rfoot(2);
                X(1,5) = X__rfoot + 0.12;   Y(1,5) = Y_rfoot + 0.025;
                X(1,6) = X__rfoot + 0.12;   Y(1,6) = Y_rfoot - 0.025;
                X(1,7) = X__rfoot - 0.06;   Y(1,7) = Y_rfoot - 0.025;
                X(1,8) = X__rfoot - 0.06;   Y(1,8) = Y_rfoot + 0.025;
            end
            if ( (feet_on_ground(1) == 0) || (feet_on_ground(2) == 0) )
                %if only  left foot is on the ground the ref frame is l_sole
                %if only right foot is on the ground the ref frame is r_sole
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

            if ~isempty( find(SubStrFind('trunk', list_of_kin_chain), 1)) )
                if ~isempty(varargin)
                    torsoInit = joints_initial_values{1,find(SubStrFind('trunk', list_of_kin_chain), 1)};
                else
                    torsoInit = [0.0;  0.0;  0.0];
                end
                string_search = {'torso_yaw', 'torso_roll', 'torso_pitch'};
                qjInit = obj.sortJointValue(string_search, qjInit, torsoInit);
            end

            if ~isempty( find(SubStrFind('left_arm', list_of_kin_chain), 1) )
                if ~isempty(varargin)
                    leftArmInit = joints_initial_values{1,find(SubStrFind('left_arm', list_of_kin_chain), 1)};
                else
                    leftArmInit = [-20.0  30.0  0.0  45.0  0.0 0.0 0.0];
                end
                string_search = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', ...
                                 'l_elbow', 'l_wrist_prosup', 'l_wrist_pitch', 'l_wrist_yaw'};
                qjInit = obj.sortJointValue(string_search,qjInit,leftArmInit);
            end

            if(~isempty(find(SubStrFind('right_arm',list_of_kin_chain),1)))
                if (~isempty(varargin))
                    rightArmInit = joints_initial_values{1,find(SubStrFind('right_arm', list_of_kin_chain), 1)};
                else
                    rightArmInit = [-20.0  30.0  0.0  45.0  0.0 0.0 0.0];
                end
                string_search = {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', ...
                                 'r_elbow','r_wrist_prosup','r_wrist_pitch','r_wrist_yaw'};
                qjInit = obj.sortJointValue(string_search, qjInit, rightArmInit);
            end

            if ( ~isempty(find(SubStrFind('l_sole', list_of_kin_chain), 1)) || ~isempty(find(SubStrFind('r_sole', list_of_kin_chain), 1)) )
                if ~isempty(varargin)
                    if ~isempty(find(SubStrFind('l_sole',list_of_kin_chain),1))
                        leftLegInit = joints_initial_values{1,find(SubStrFind('l_sole', list_of_kin_chain), 1)};
                    end
                    if ~isempty(find(SubStrFind('r_sole',list_of_kin_chain),1))
                        rightLegInit = joints_initial_values{1,find(SubStrFind('r_sole', list_of_kin_chain), 1)};
                    end
                else
                    if feet_on_ground(1) == 1 && feet_on_ground(2) == 1
                        % initial conditions for balancing on two feet
                        leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1];
                        rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1];
                    elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 0
                        % initial conditions for the robot standing on the left foot
                        leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1];
                        rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1];
                    elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
                        % initial conditions for the robot standing on the right foot
                        leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  -0.1];
                        rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  -0.1];
                    end
                end
                string_search = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw','l_knee', ...
                                 'l_ankle_pitch', 'l_ankle_roll'};
                qjInit = obj.sortJointValue(string_search,qjInit,leftLegInit);
                string_search = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', ...
                                 'r_ankle_pitch', 'r_ankle_roll'};

                qjInit = obj.sortJointValue(string_search, qjInit, rightLegInit);
            end
            qjInit = qjInit.*(pi/180);
        end

        % Same as the method fkine but can be call at anytime. By tag you
        % specify through a string the name of the joint you want
        function [x,R] = offlineFkine(obj, chi, tag)
            x_base = chi(1:3,:); %TODO floating base flag required (parameter of the simulator)
            qt_b   = chi(4:7,:);    %TODO floating base flag required (parameter of the simulator)
            q      = chi(8:8+obj.ndof-1,:);
            %[~,R_base] = frame2posrot([x_base; qt_b]);
            [~,R_base] = WBM.utilities.frame2posRotm([x_base; qt_b]);

            % fkine = wholeBodyModel('forward-kinematics', reshape(R_base,[],1), x_base, q, tag);
            fkine = obj.mwbm_icub.forwardKinematics(R_base, x_base, q, tag);

            % Obtaining the rotation matrix from root link to world frame
            %[x,R] = frame2posrot(fkine);
            [x,R] = WBM.utilities.frame2posRotm(fkine);
        end

        % Same as the method jacob0 but can be call at anytime. By tag you
        % specify through a string the name of the joint you want
        function jacob0 = offlineJacob0(obj, chi, tag)
            x_base = chi(1:3,:);  %TODO floating base flag required (parameter of the simulator)
            qt_b   = chi(4:7,:);  %TODO floating base flag required (parameter of the simulator)
            q      = chi(8:8+obj.ndof-1,:);
            %[~,R_base] = frame2posrot([x_base;qt_b]);
            [~,R_base] = WBM.utilities.frame2posRotm([x_base; qt_b]);

            % jacob0 = wholeBodyModel('jacobian', reshape(R_base,[],1), x_base, q, tag);
            jacob0 = obj.mwbm_icub.jacobian(R_base, x_base, q, tag);
        end

        % Create the constraints_values vector need to compute the constraints
        % Used in AllRUntimeParameters
        function vector = createConstraintsVector(obj)
            vector = [];
            for i = 1:obj.ndof
                vector = [vector, obj.UBjointLimit(i), obj.LBjointLimit(i)];
            end

            for i = 1:obj.ndof
                vector = [vector, obj.effortLimit(i), -obj.effortLimit(i)];
            end
        end

        [] = plot(obj, chi, params, varargin)

    end
end
