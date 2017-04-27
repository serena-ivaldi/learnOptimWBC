classdef iCub < handle
    
    properties
        %% Structural Parameters
        model_name
        %active_floating_base         % switch to control if the icub has %or not a floating base i want to use param for this variable
        ndof                          % dependant on the model used for the simulaztion
        %list_of_kin_chain             % string matching URDF name of the link (frame)
        %dim_of_kin_chain
        %access_index;                 % vector of index used for accesing the infromation stored in the structural parameters
        %% Whole body dynamic parameters and state
        init_state          % w_R_b,x_b,q,qd,dx_b,w_omega_b
        state               % w_R_b,x_b,q,qd,dx_b,w_omega_b
        dynamic             % M,h,g,H,C_nu,JCoM,dJCoM_nu,Jc,dJc_nu,JH,dJH_nu;       
        %% kinematic information
        contact_jacobians
        support_poly
        %% URDF parameter   (only for urdf model for the IcubGazeboSim model i wil not consider this value)
        jointList           % the list of all the joints in the same order than the urdf
        linkList            % the list of all the links in the same order than the urdf
        revoluteJointList   % the list of all the not fixed joints in the same order than the urdf
        UBjointLimit        % Upper limit boundarie of all the revolute joints
        LBjointLimit        % Lower limit boundarie of all the revolute joints
        effortLimit         % Limit efforts of all the revolute joints
        %% Enanched visualization field
        modelName   
        mdlLdr           
        consideredJoints
        setPos               
        setCamera
        lightDir
    end
    
    methods
        %% initialization functions
        function obj = iCub(model)
            if(strcmp(model,'icubGazeboSim'))
                obj.model_name = 'model';
                %obj.active_floating_base = true;
                % Initialize the mexWholeBodyModel
                wbm_modelInitialize('icubGazeboSim');
                wbm_resetWorldFrame();
                obj.ndof = 25; % degrees of freedom with floating base
                % fixed joint list for icubGazeboSim
                obj.revoluteJointList = {'torso_pitch','torso_roll','torso_yaw','l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow','l_wrist_prosup','r_shoulder_pitch','r_shoulder_roll',...
                                  'r_shoulder_yaw','r_elbow','r_wrist_prosup','l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll','r_hip_pitch','r_hip_roll',...
                                  'r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
                              
                model = strcat(obj.model_name,'.urdf');
                path = which(model);              
                scheme = xml2struct(path);
                obj.UBjointLimit = zeros(1,obj.ndof);
                obj.LBjointLimit = zeros(1,obj.ndof);
                obj.effortLimit  = zeros(1,obj.ndof);
                iii = 1;
                for i = 1:length(scheme.robot.link)
                    if(~strcmp(scheme.robot.link{i}.Attributes.name,'base_link'))
                        obj.linkList{iii} = scheme.robot.link{i}.Attributes.name;
                        iii = iii + 1;
                    end
                end
                for i = 1:length(scheme.robot.joint)
                    obj.jointList{i} = scheme.robot.joint{i};
                    current_joint = scheme.robot.joint{i}.Attributes.name;
                    index = strcmp(current_joint, obj.revoluteJointList);
                    if  ( sum(index) )
                        obj.UBjointLimit(index) =  str2double(scheme.robot.joint{i}.limit.Attributes.upper);
                        obj.LBjointLimit(index) =  str2double(scheme.robot.joint{i}.limit.Attributes.lower);
                        obj.effortLimit(index)  =  str2double(scheme.robot.joint{i}.limit.Attributes.effort);
                    end
                end              
            else
                obj.model_name = model;
                model = strcat(model,'.urdf');
                path = which(model);
                wbm_modelInitializeFromURDF(path);
                wbm_resetWorldFrame();
                scheme = xml2struct(path);
                obj.UBjointLimit = [];
                obj.LBjointLimit = [];
                obj.effortLimit = [];
                iii = 1;
                for i = 1:length(scheme.robot.link)
                    if(~strcmp(scheme.robot.link{i}.Attributes.name,'base_link'))
                        obj.linkList{iii} = scheme.robot.link{i}.Attributes.name;
                        iii = iii + 1;
                    end
                end
                iii = 1;
                for i = 1:length(scheme.robot.joint)
                    obj.jointList{i} = scheme.robot.joint{i};
                    if  strcmp(obj.jointList{i}.Attributes.type,'revolute')
                        obj.revoluteJointList{iii} = scheme.robot.joint{i};
                        obj.UBjointLimit = [obj.UBjointLimit, str2double(scheme.robot.joint{i}.limit.Attributes.upper)];
                        obj.LBjointLimit = [obj.LBjointLimit, str2double(scheme.robot.joint{i}.limit.Attributes.lower)];
                        obj.effortLimit = [obj.effortLimit, str2double(scheme.robot.joint{i}.limit.Attributes.effort)];
                        iii = iii +1;
                    end
                end
                obj.ndof = length(obj.revoluteJointList);
            end
            
            obj.InitEnhanViz();
            
        end
        
        function InitEnhanViz(obj)
            obj.modelName        = 'iCub';
            obj.setPos           = [1,0,0.5];    
            obj.setCamera        = [0.4,0,0.5];%[0.4,0,0.5];
            obj.mdlLdr           = iDynTree.ModelLoader();
            obj.consideredJoints = iDynTree.StringVector();

            obj.consideredJoints.push_back('torso_pitch');
            obj.consideredJoints.push_back('torso_roll');
            obj.consideredJoints.push_back('torso_yaw');
            obj.consideredJoints.push_back('l_shoulder_pitch');
            obj.consideredJoints.push_back('l_shoulder_roll');
            obj.consideredJoints.push_back('l_shoulder_yaw');
            obj.consideredJoints.push_back('l_elbow');
            obj.consideredJoints.push_back('l_wrist_prosup');
            obj.consideredJoints.push_back('r_shoulder_pitch');
            obj.consideredJoints.push_back('r_shoulder_roll');
            obj.consideredJoints.push_back('r_shoulder_yaw');
            obj.consideredJoints.push_back('r_elbow');
            obj.consideredJoints.push_back('r_wrist_prosup');
            obj.consideredJoints.push_back('l_hip_pitch');
            obj.consideredJoints.push_back('l_hip_roll');
            obj.consideredJoints.push_back('l_hip_yaw');
            obj.consideredJoints.push_back('l_knee');
            obj.consideredJoints.push_back('l_ankle_pitch');
            obj.consideredJoints.push_back('l_ankle_roll');
            obj.consideredJoints.push_back('r_hip_pitch');
            obj.consideredJoints.push_back('r_hip_roll');
            obj.consideredJoints.push_back('r_hip_yaw');
            obj.consideredJoints.push_back('r_knee');
            obj.consideredJoints.push_back('r_ankle_pitch');
            obj.consideredJoints.push_back('r_ankle_roll');
            
            urdf_model_name = 'model.urdf';
            allpath = which('Icub_model_placeholder.m');
            path =fileparts(allpath);
            path = strcat(path,'/',urdf_model_name);
            obj.mdlLdr.loadReducedModelFromFile(path,obj.consideredJoints);
            
            % set lights
            obj.lightDir = iDynTree.Direction();     
            obj.lightDir.fromMatlab([-0.5 0 -0.5]/sqrt(2));    
        end
        
        function qjInit = InitializeStateicubGazeboSim(obj,feet_on_ground)
            %% Initial joints position [deg]
            % lifted arm      [ -20  30  0  45  0]
            % stretched arm   [  20  30  0  45  0]
            leftArmInit  = [  5  30  0  45  0]'; 
            rightArmInit = [  5  30  0  45  0]';
            torsoInit    = [  60   0  0]';
            
            if sum(feet_on_ground) >= 2
                % initial conditions for balancing on two feet
                 leftLegInit  = [  90   0   0  -90  -10.5  0]';
                 rightLegInit = [  90   0   0  -90  -10.5  0]';
%                leftLegInit  = [  90   0   0  -90  -5.5  0]';
%                rightLegInit = [  90   0   0  -90  -5.5  0]';
            elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 0

                % initial conditions for the robot standing on the left foot
                leftLegInit  = [  25.5   15   0  -18.5  -5.5  0]';
                rightLegInit = [  25.5    5   0  -40    -5.5  0]';

            elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1

                % initial conditions for the robot standing on the right foot
                leftLegInit  = [  25.5    5   0  -40    -5.5  0]';
                rightLegInit = [  25.5   15   0  -18.5  -5.5  0]';
            end
            
            % joints configuration [rad]
            qjInit    = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);
            obj.init_state.qi = qjInit;
        end
        
         % This function initialize and allow to use different limbs of the
        % robot according to the current URDF
        % The possible values of kin_chain are
        % 'trunk','left_arm','right_arm','l_sole','r_sole'
        % return qjInit in degres
        % If no initial joints values are passed through varargin the
        % function automatically set iCub to the default position
        function qjInit = InitializeStateUrdf(obj, list_of_kin_chain, feet_on_ground, varargin)
            qjInit = zeros(obj.ndof,1);
            if (~isempty(varargin))
                joints_initial_values  = varargin{1};
                if ~(length(joints_initial_values) == length(list_of_kin_chain))
                    error('joints_initial_values is malformed, should be the same lenght as list_of_kin_chain');
                end
            end
            
            if(~isempty(find(SubStrFind('trunk',list_of_kin_chain),1)))
                if (~isempty(varargin))
                    torsoInit    = joints_initial_values{1,find(SubStrFind('trunk',list_of_kin_chain),1)};
                else
                    torsoInit    = [0.0  0.0  0.0];
                end
                % here the hyp is that i can assign the starting position
                % by following the order specified in the string_search
                % variables than i will apply each value in the right
                % positions through sortJointValue function. the idea is
                % that the joint vector is in the same order of joints in
                % in the xml file
                string_search = {'torso_yaw','torso_roll','torso_pitch'};
                qjInit = obj.sortJointValue(string_search,qjInit,torsoInit);
            end
            
            if(~isempty(find(SubStrFind('left_arm',list_of_kin_chain),1)))
                if (~isempty(varargin))
                    leftArmInit    = joints_initial_values{1,find(SubStrFind('left_arm',list_of_kin_chain),1)};
                else
                    leftArmInit  = [-20.0  30.0  0.0  45.0  0.0 0.0 0.0];%0.0  5.0  0.0  10.0  0.0 0.0 0.0
                end
                string_search = {'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw',...
                    'l_elbow','l_wrist_prosup','l_wrist_pitch','l_wrist_yaw'};
                qjInit = obj.sortJointValue(string_search,qjInit,leftArmInit);
            end
            
            if(~isempty(find(SubStrFind('right_arm',list_of_kin_chain),1)))
                if (~isempty(varargin))
                    rightArmInit    = joints_initial_values{1,find(SubStrFind('right_arm',list_of_kin_chain),1)};
                else
                    rightArmInit = [-20.0  30.0  0.0  45.0  0.0 0.0 0.0];
                end
                string_search = {'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw',...
                    'r_elbow','r_wrist_prosup','r_wrist_pitch','r_wrist_yaw'};
                qjInit = obj.sortJointValue(string_search,qjInit,rightArmInit);
            end
            
            if(~isempty(find(SubStrFind('l_sole',list_of_kin_chain),1)) || ~isempty(find(SubStrFind('r_sole',list_of_kin_chain),1)))
                if (~isempty(varargin))
                    if(~isempty(find(SubStrFind('l_sole',list_of_kin_chain),1)))
                        leftLegInit    = joints_initial_values{1,find(SubStrFind('l_sole',list_of_kin_chain),1)};
                    end
                    if(~isempty(find(SubStrFind('r_sole',list_of_kin_chain),1)))
                        rightLegInit    = joints_initial_values{1,find(SubStrFind('r_sole',list_of_kin_chain),1)};
                    end
                else
                    if     feet_on_ground(1) == 1 && feet_on_ground(2) == 1
                        % initial conditions for balancing on two feet
                        leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1];
                        rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1];
                    elseif   feet_on_ground(1) == 1 && feet_on_ground(2) == 0
                        % initial conditions for the robot standing on the left foot
                        leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1];
                        rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1];
                    elseif   feet_on_ground(1) == 0 && feet_on_ground(2) == 1
                        % initial conditions for the robot standing on the right foot
                        leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  -0.1];
                        rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  -0.1];
                    end
                end
                string_search = {'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee',...
                    'l_ankle_pitch','l_ankle_roll'};
                qjInit = obj.sortJointValue(string_search,qjInit,leftLegInit);
                string_search = {'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee',...
                    'r_ankle_pitch','r_ankle_roll'};
                qjInit = obj.sortJointValue(string_search,qjInit,rightLegInit);
            end
            qjInit = qjInit.*(pi/180);
            
        end
        
        %% State function
        
        function [state,x_b,qt_b,w_R_b,base_pose,q,dx_b,w_omega_b,qd,Nu]=State(obj,chi)
            import WBM.utilities.frame2posRotm;
            %% TODO floating base flag required (parameter of the simulator)
            x_b  = chi(1:3,:);                         % floating base position
            qt_b = chi(4:7,:);                         % floating base orientation (with quaternion)
            base_pose = [x_b;qt_b];                    % base velocity position and orientation 
            % normalize quaternions to avoid numerical errors
            % qt_b = qt_b/norm(qt_b);
            q   = chi(8:8+obj.ndof-1,:);
            % linear and angular velocity
            dx_b    = chi(obj.ndof+8:obj.ndof+10,:);    % floating base linear velocity
            w_omega_b = chi(obj.ndof+11:obj.ndof+13,:);   % floating base angular velocity
            qd     = chi(obj.ndof+14:2*obj.ndof+13,:); % joint velocity
            Nu      = [dx_b;w_omega_b;qd];               % velocity vector
            % Obtaining the rotation matrix from root link to world                 
            [~,w_R_b]    = frame2posRotm(base_pose);
            %% Define the output structure
            obj.state.q                          = q;
            obj.state.qd                         = qd;
            obj.state.Nu                         = Nu;
            obj.state.basePose                   = base_pose;
            obj.state.w_R_b                      = w_R_b;
            obj.state.x_b                        = x_b;
            obj.state.dx_b                       = dx_b;
            obj.state.qt_b                       = qt_b;
            obj.state.w_omega_b                  = w_omega_b; 
            % return 
            state = obj.state;
        end
        
        function SetWorldFrameiCub(obj,qjInit,dqjInit,dx_bInit,w_omega_bInit,reference_link)
            % Updating the robot position
            wbm_updateState(qjInit,dqjInit,[dx_bInit;w_omega_bInit]);
            % fixing the world reference frame w.r.t. the foot on ground position
            [x_b0,w_R_b0] = wbm_getWorldFrameFromFixLnk(reference_link,qjInit);
            % define world frame
            wbm_setWorldFrame(w_R_b0,x_b0,[0 0 -9.81]');
                      
            % update position and orientation of the floating base repect of the root base
            obj.state.q = qjInit;
            obj.state.qd = dqjInit;
            obj.state.x_b = x_b0;
            obj.state.w_R_b = w_R_b0;
            % initial velocity floating base
            obj.state.dx_b   = dx_bInit;
            obj.state.w_omega_b = w_omega_bInit;
            
            obj.init_state.qi = qjInit;
            obj.init_state.qdi = dqjInit;
            obj.init_state.x_bi = x_b0;
            obj.init_state.w_R_bi = w_R_b0;
            obj.init_state.dx_bi   = dx_bInit;
            obj.init_state.w_omega_bi = w_omega_bInit;
            %% TODEBUG
            % solo per adesso per la integrazione del balance
            Xcom_pose = wbm_forwardKinematics(obj.init_state.w_R_bi,obj.init_state.x_bi,obj.init_state.qi,'com');
            obj.init_state.xCoMRef = Xcom_pose(1:3);
        end
        
        %   qj  - joint angles (NumDoF x 1)
        %   xTb - base rototranslation (7 x 1) with 3 for position of frame and 4 for orientation quaternion
        %         (return is already flipped so that quaternion organised as real
        %         followed by imaginary)
        %   dqj - joint velocities (NumDoF x 1)
        %   vxb - floating base velocity (6 x 1)
        function [qj,xTb,qjDot,vb] = GetState(obj)
            [xTb,qj,vb,qjDot]=wbm_getState();
        end
        
        %x_b        position of the floating base
        %qt_b       orientation fo the floating base with quaternion
        %dx_b       linear velocity of the floating base
        %omega_w    angular velocity of the floating base in world frame
        function  SetFloatingBaseState(obj,x_b,qt_b,dx_b,w_omega_b)
            import WBM.utilities.frame2posRotm; 
            obj.state.x_b = x_b;
            obj.state.dx_b = dx_b;
            obj.state.w_omega_b = w_omega_b;
            % Obtaining the rotation matrix from root link to world frame
            qT         = [x_b;qt_b];
            [~,obj.state.w_R_b]    = frame2posRotm(qT);
        end
        
        %% dynamics functions
        function [dynamic,M,h,g,H,C_nu,JCoM,dJCoM_nu,JH,dJH_nu]=Dynamics(obj)
           q  = obj.state.q;
           qd = obj.state.qd;
           w_R_b = obj.state.w_R_b;
           w_omega_b = obj.state.w_omega_b;
           x_b = obj.state.x_b;
           dx_b = obj.state.dx_b;
           
           % dynamic quantity
           M = obj.inertia(q);
           
           [C_nu ,h, g] = obj.coriolis(q,qd);
           % Jacobians and dJ_nu
           JCoM = wbm_jacobian(w_R_b,x_b,q,'com');
           dJCoM_nu = wbm_dJdq(w_R_b,x_b,q,qd,[dx_b;w_omega_b],'com');
           % centroidal momemntum 
           H = obj.CentroidalMomentum(q,qd);
           % centroidal momentum Jacobian
           JHBase   = zeros(6,6);
           JHJoint  = zeros(6,obj.ndof);
           for ii = 1:6
               v_bJH         = zeros(6,1);
               v_bJH(ii)     = 1;
               JHBase(:,ii)  = wbm_centroidalMomentum(w_R_b,x_b,q,zeros(obj.ndof,1),v_bJH);
           end

           for ii = 1:obj.ndof
               dqjJH          = zeros(obj.ndof,1);
               dqjJH(ii)      = 1;
               JHJoint(:,ii)  = wbm_centroidalMomentum(w_R_b,x_b,q,dqjJH,zeros(6,1));
           end
           JH      = [JHBase JHJoint];
           dJH_nu  = [M(1,1)*dJCoM_nu(1:3); zeros(3,1)];
           
           obj.dynamic.M          = M;
           obj.dynamic.h          = h;
           obj.dynamic.g          = g;
           obj.dynamic.H          = H;
           obj.dynamic.C_nu       = C_nu;
           obj.dynamic.JCoM       = JCoM;
           obj.dynamic.dJCoM_nu   = dJCoM_nu;
           obj.dynamic.JH         = JH;
           obj.dynamic.dJH_nu     = dJH_nu;
           % return
           dynamic = obj.dynamic;
        end
        
        function [M,F,Omega]=WholeBodyDynamics(obj,q,qd)
            M = obj.inertia(q);
            F = wbm_generalizedBiasForces(obj.state.w_R_b,obj.state.x_b,q,qd,[obj.state.dx_b;obj.state.w_omega_b]);
            Omega = obj.CentroidalMomentum(q,qd);
        end
        
        % the hypothesis is that fc is already premultiplied by the Jc (contact jacobian)
        function f = F(obj,q,qd,fc,Jc_t)
            f = wbm_generalisedBiasForces(obj.state.w_R_b,obj.state.x_b,q,qd,[obj.state.dx_b;obj.state.w_omega_b]) -Jc_t*fc;
        end
        %% kinematic functions
        function [contact_jacobians,Jc_sym,dJcNu_sym]=ContactJacobians(obj,param,contact)
            q  = obj.state.q;
            qd = obj.state.qd;
            % contact jacobians for controller
            Jc    = zeros(6*param.numContacts,6+obj.ndof);
            dJcNu = zeros(6*param.numContacts,1);
            for i=1:param.numContacts
                Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(obj.state.w_R_b,obj.state.x_b,q,param.contactLinkNames{i});
                dJcNu(6*(i-1)+1:6*i,:) = wbm_dJdq(obj.state.w_R_b,obj.state.x_b,q,qd,[obj.state.dx_b;obj.state.w_omega_b],param.contactLinkNames{i});
            end
            % i had to put this correction because for some reason the
            % r_sole does not work properly
            dJcNu(9) = 0;
            % contact jacobian for simulator
            Jc_sym       =  zeros(6*contact.num_of_active_contacts,6+obj.ndof);
            dJcNu_sym    =  zeros(6*contact.num_of_active_contacts,1);
            for i=1:contact.num_of_active_contacts
                Jc_sym(6*(i-1)+1:6*i,:)    = wbm_jacobian(obj.state.w_R_b,obj.state.x_b,q,contact.names{i});
                dJcNu_sym(6*(i-1)+1:6*i,:) = wbm_dJdq(obj.state.w_R_b,obj.state.x_b,q,qd,[obj.state.dx_b;obj.state.w_omega_b],contact.names{i});
            end
            % i had to put this correction because for some reason the
            % r_sole does not work properly and give me a non zero element
            % for the r_sole
            dJcNu_sym(9) = 0;
            obj.contact_jacobians.Jc = Jc;
            obj.contact_jacobians.dJcNu = dJcNu;
            contact_jacobians = obj.contact_jacobians;
        end
        
        % Same as the method fkine but can be call at anytime. By tag you
        % specify through a string the name of the joint you want
        function  [x,R] = offlineFkine(rob,chi,tag)
        import WBM.utilities.frame2posRotm;    
            x_base  = chi(1:3,:);  %TODO floating base flag required (parameter of the simulator)
            qt_b = chi(4:7,:);  %TODO floating base flag required (parameter of the simulator)
            q   = chi(8:8+rob.ndof-1,:);
            [~,R_base]    = frame2posRotm([x_base;qt_b]);
            fkine = wholeBodyModel('forward-kinematics',reshape(R_base,[],1),x_base,q,tag);
            % Obtaining the rotation matrix from root link to world frame
            [x,R]    = frame2posrot(fkine);
        end
        % Same as the method jacob0 but can be call at anytime. By tag you
        % specify through a string the name of the joint you want
        function  jacob0 = offlineJacob0(rob,chi,tag)
            x_base  = chi(1:3,:);  %TODO floating base flag required (parameter of the simulator)
            qt_b = chi(4:7,:);  %TODO floating base flag required (parameter of the simulator)
            q   = chi(8:8+rob.ndof-1,:);
            [~,R_base]    = frame2posrot([x_base;qt_b]);
            jacob0 = wholeBodyModel('jacobian',reshape(R_base,[],1),x_base,q,tag);
        end
        
        
        %% miscellanea
%         function suppConvHull = computeSupPoly(obj, feet_on_ground,chi)
%         %Compute the support polygone wrt feet_on_ground
%             if     feet_on_ground(1) == 1 && feet_on_ground(2) == 1
%                 %if both feet on the ground the ref frame is l_sole
%                 %l_foot
%                 [pos_lfoot,~]    = obj.offlineFkine(chi,'r_sole');
%                 X__lfoot = pos_lfoot(1);    Y_lfoot = pos_lfoot(2);
%                 X(1,1) = X__lfoot + 0.12;      Y(1,1) = Y_lfoot + 0.025;
%                 X(1,2) = X__lfoot + 0.12;      Y(1,2) = Y_lfoot -0.025;
%                 X(1,3) = X__lfoot - 0.06;     Y(1,3) = Y_lfoot - 0.025;
%                 X(1,4) = X__lfoot - 0.06;     Y(1,4) = Y_lfoot + 0.025;
%                 %r_foot
%                 [pos_rfoot,~]    = obj.offlineFkine(chi,'l_sole');
%                 X__rfoot = pos_rfoot(1);    Y_rfoot = pos_rfoot(2);
%                 X(1,5) = X__rfoot + 0.12;     Y(1,5) = Y_rfoot + 0.025;
%                 X(1,6) = X__rfoot + 0.12;     Y(1,6) = Y_rfoot - 0.025;
%                 X(1,7) = X__rfoot - 0.06;     Y(1,7) = Y_rfoot - 0.025;
%                 X(1,8) = X__rfoot - 0.06;     Y(1,8) = Y_rfoot + 0.025;
%             end
%             if     feet_on_ground(1) == 0 || feet_on_ground(2) == 0
%                 %if only  left foot is on the ground the ref frame is l_sole
%                 %if only right foot is on the ground the ref frame is r_sole
%                 X(1,1) = 0.12;      Y(1,1) = 0.025;
%                 X(1,2) = 0.12;      Y(1,2) = -0.025;
%                 X(1,3) = -0.06;     Y(1,3) = -0.025;
%                 X(1,4) = -0.06;     Y(1,4) = 0.025;
%             end
%             suppConvHull = ConvexHull(X,Y);
%         end


% %           right_foot      left_foot
% %         C  _______  B   C' _______ B'          
% %           |       |       |       |   Y_w
% %           |       |       |   ---------->
% %           |       |       |   |   |
% %         D |_______| A   D'|___|__ | A'
% %                               |
% %                           X_w |
% %                               v
% %         double support :   
% %             support_poly.min = C
% %             support_poly.max = A'
% %         single support  left foot:
% %             support_poly.min = C'
% %             support_poly.max = A'
% %         single support  left foot:
% %             support_poly.min = C
% %             support_poly.max = A
                                
        function ComputeSupportPoly(obj,params)
             import WBM.utilities.frame2posRotm; 
             l_sole_pos   = wbm_forwardKinematics(obj.state.w_R_b,obj.state.x_b,obj.state.q,'l_sole');
             r_sole_pos   = wbm_forwardKinematics(obj.state.w_R_b,obj.state.x_b,obj.state.q,'r_sole');
             
             [x_l_sole,~] = frame2posRotm(l_sole_pos);
             [x_r_sole,~] = frame2posRotm(r_sole_pos);
             
             
             shift_mult = params.footSize(1) - 0.03;
             x_dir = params.footSize(1)*[1,0];
             y_dir = params.footSize(2)*[0,1];
             x_shift = shift_mult* [1 0];
             
             
             if  params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1 
                 
                 obj.support_poly.min =  x_r_sole(1:2)' - x_dir - y_dir + x_shift;
                 obj.support_poly.max =  x_l_sole(1:2)' + x_dir + y_dir + x_shift;
                 
             elseif params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0 
                 
                 obj.support_poly.min =  x_l_sole(1:2)' - x_dir - y_dir + x_shift;
                 obj.support_poly.max =  x_l_sole(1:2)' + x_dir + y_dir + x_shift;
                 
             elseif params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
                 
                 obj.support_poly.min =  x_r_sole(1:2)' - x_dir - y_dir + x_shift;
                 obj.support_poly.max =  x_r_sole(1:2)' + x_dir + y_dir + x_shift;
                 
             end
             
             obj.support_poly.center = (obj.support_poly.min + obj.support_poly.max)/2;
             obj.support_poly.height =  obj.support_poly.max(1) - obj.support_poly.min(1);
             obj.support_poly.width  =  obj.support_poly.max(2) - obj.support_poly.min(2); 
             obj.support_poly.max_dist = norm(obj.support_poly.max - obj.support_poly.center);
        end

        
        % Create the constraints_values vector need to compute the constraints
        % Used in AllRUntimeParameters
        function vector = createConstraintsVector(obj)
            vector = [];
            for i = 1:obj.ndof
                                   %upper bound                %lower bound  
                vector = [vector, obj.UBjointLimit(i), obj.LBjointLimit(i)];
            end
            for i = 1:obj.ndof
                vector = [vector, obj.effortLimit(i), -obj.effortLimit(i)];
            end
        end
    end
end
