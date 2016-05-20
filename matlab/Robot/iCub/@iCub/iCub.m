classdef iCub < handle
    
    properties
        %% Structural Parameters
        model_name
        active_floating_base          % switch to control if the icub has or not a floating base
        ndof                          % dependant on the model used for the simulaztion
        list_of_kin_chain             % string matching URDF name of the link (frame)
        %dim_of_kin_chain
        %access_index;                 % vector of index used for accesing the infromation stored in the structural parameters
        %% Floating base parameter
        % I use this value inside the computation of the dynamical
        % parameters of the robot (i need to udpate them each iteration)
        x_b      %the cartesian position of the base (R^3)
        R_b      %the rotation matrix describing the orientation of the base (global parametrization of SO(3))
        dx_b     %the cartesian velocity of the base (R^3)
        omega_W  %the velocity describing the orientation of the base (SO(3))
        %% Whole body dynamic parameters
        %M
        %F
        %Omega
        %kinematic_chain_selector % list of kinematic chain in the icub
        %% URDF parameter
        jointList   %the list of all the joints in the same order than the urdf
        linkList    %the list of all the links in the same order than the urdf
        revoluteJointList %the list of all the not fixed joints in the same order than the urdf
    end
    
    methods
        function obj = iCub(model,list_of_kin_chain)
            if(strcmp(model,'icubGazeboSim'))
                obj.model_name = model;
                obj.active_floating_base = true;
                % Initialize the mexWholeBodyModel
                wbm_modelInitialise('icubGazeboSim');
                obj.list_of_kin_chain = list_of_kin_chain; %{'com','left_arm','right_arm','l_sole','r_sole'}; %string matching URDF name of the link (frame)
                %obj.dim_of_kin_chain  = {3,5,5,6,6};
                %obj.sum_ind = {0,3,8,13,19,25};
                obj.ndof = 25; % degrees of freedom without floating base
            elseif(strcmp(model,'model_arms_torso_free')||strcmp(model,'model32dof'))
                if strcmp(model,'model32dof') 
                    obj.ndof = 32;
                else
                    obj.ndof = 17;
                end
                obj.model_name = model;
                obj.active_floating_base = false;
                obj.list_of_kin_chain =list_of_kin_chain; % {'com','left_arm','right_arm'}; %string matching URDF name of the link (frame)
                model = strcat(model,'.urdf');
                path = which(model);
                wbm_modelInitialiseFromURDF(path);
                scheme = xml2struct(path);
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
                        iii = iii +1;
                    end
                end
            end
        end
        
        function SetWorldFrameiCub(obj,qjInit,dqjInit,dx_bInit,omega_WInit,reference_link)
            %% Updating the robot position
            wbm_updateState(qjInit,dqjInit,[dx_bInit;omega_WInit]);
            % fixing the world reference frame w.r.t. the foot on ground position
            [R_b0,x_b0] = wbm_getWorldFrameFromFixedLink(reference_link,qjInit);
            % define world frame
            wbm_setWorldFrame(R_b0,x_b0,[0 0 -9.81]');
            % update position and orientation of the floating base repect of the root base
            obj.x_b = x_b0;
            obj.R_b = R_b0;
            % initial velocity floating base
            obj.dx_b   = dx_bInit;
            obj.omega_W = omega_WInit;
        end
        
        %   qj  - joint angles (NumDoF x 1)
        %   xTb - base rototranslation (7 x 1) with 3 for position of frame and 4 for orientation quaternion
        %         (return is already flipped so that quaternion organised as real
        %         followed by imaginary)
        %   dqj - joint velocities (NumDoF x 1)
        %   vxb - floating base velocity (6 x 1)
        function [qj,xTb,qjDot,vb] = GetState(obj)
            [qj,xTb,qjDot,vb]=wbm_getState();
        end
        
        %x_b        position of the floating base
        %qt_b       orientation fo the floating base with quaternion
        %dx_b       linear velocity of the floating base
        %omega_w    angular velocity of the floating base in world frame
        function  SetFloatingBaseState(obj,x_b,qt_b,dx_b,omega_W)
            obj.x_b = x_b;
            obj.dx_b = dx_b;
            obj.omega_W = omega_W;
            % Obtaining the rotation matrix from root link to world frame
            qT         = [x_b;qt_b];
            [~,obj.R_b]    = frame2posrot(qT);
        end
        
        function [M,F,Omega]=WholeBodyDynamics(obj,q,qd)
            M = obj.inertia(q);
            F = wbm_generalisedBiasForces(obj.R_b,obj.x_b,q,qd,[obj.dx_b;obj.omega_W]);
            Omega = obj.CentroidalMomentum(q,qd);
        end
        
        % the hypothesis is that fc is already premultiplied by the Jc (contact jacobian)
        function f = F(obj,q,qd,fc,Jc_t)
            f = wbm_generalisedBiasForces(obj.R_b,obj.x_b,q,qd,[obj.dx_b;obj.omega_W]) -Jc_t*fc;
        end

        end
  
end
