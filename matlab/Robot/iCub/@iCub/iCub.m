classdef iCub < handle
 
	properties     
        %% Structural Parameters
        ndof   % dependant on the model used for the simulaztion
        list_of_kin_chain  %string matching URDF name of the link (frame)
        dim_of_kin_chain 
        access_index; % vector of index used for accesing the infromation stored in the structural parameters
        %% Floating base parameter
        % I use this value inside the computation of the dynamical
        % parameters of the robot (i need to udpate them each iteration)
        x_b      %the cartesian position of the base (R^3)
        R_b     %the quaternion describing the orientation of the base (global parametrization of SO(3))
        dx_b     %the cartesian velocity of the base (R^3)
        omega_w  %the velocity describing the orientation of the base (SO(3))
        %% Whole body dynamic parameters
        M
        F
        Omega
        %% for visualization
        visual_param;
        
        %kinematic_chain_selector % list of kinematic chain in the icub
        %cur_chain                % current chain that we want to control
    end
	
	methods
		function ro = iCub(obj,model)
            if(strcmp(model,'icubGazeboSim'))
                % Initialize the mexWholeBodyModel
                wbm_modelInitialise('icubGazeboSim');	
                obj.list_of_kin_chain = {'r_sole','l_sole','com','right_arm','left_arm'}; %string matching URDF name of the link (frame)
                obj.dim_of_kin_chain  = {6,6,3,5,5};
                obj.ndof = 25;
            else
                wbm_modelInitialiseFromURDF(model);  
            end
        end
        
        function SetWorldFrameiCub(obj,qjInit,reference_link)
            %% Updating the robot position
			wbm_updateState(qjInit,dqjInit,[dx_bInit;omega_bInit]);
			% fixing the world reference frame w.r.t. the foot on ground position
			[R_b0,x_b0] = wbm_getWorldFrameFromFixedLink(reference_link,qjInit);
			% define world frame
			wbm_setWorldFrame(R_b0,x_b0,[0 0 -9.81]');
            % update position and orientation of the floating base repect of the root base 
            obj.x_b = x_b0;
            obj.R_b = R_b0;
            % initial velocity floating base
            obj.dx_b   = zeros(3,1);
            obj.omega_b = zeros(3,1);
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
        %omega_w    angular velocity of the floating base       
        function  SetFloatingBaseState(obj,x_b,qt_b,dx_b,omega_w)
            obj.x_b = x_b;
            obj.dx_b = dx_b;
            obj.omega_w = omega_w;
            % Obtaining the rotation matrix from root link to world frame
            qT         = [x_b;qt_b];
            [~,obj.R_b]    = frame2posrot(qT);
        end
        
        function WholeBodyDynamics(obj,q,qd)
            obj.M = obj.inertia(q); 
            obj.F = obj.coriolis(q,qd)*qd' + obj.gravload(q)';
            obj.Omega = CentroidalMomentum(q,qd);
        end
	end
	 
end
