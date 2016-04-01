classdef iCub < handle
 
	properties
        %% Setup params for balancing controller
        demo_movements           %=  1;                                      %either 0 or 1; only for two feet on the ground
        use_QPsolver             %=  0;                                      %either 0 or 1
        use_Orientation          %=  1;                                      %either 0 or 1
        % balancing on two feet or one foot
        feet_on_ground            %=  [1,1];                                  %either 0 or 1; [left,right]
        % allows the visualization of torques, forces and other user-defined graphics 
        visualizer_graphics      %=  1;                                      %either 0 or 1
        visualizer_demo          %=  1;                                      %either 0 or 1
        visualizer_jointsPos     %=  0;                                      %either 0 or 1; only if visualizer_graphics = 1
        %% Setup general params
        % this is assuming a 25DoF iCub
        ndof         %$= 25;
        x_b           %the cartesian position of the base (R^3)
        qt_b     %the quaternion describing the orientation of the base (global parametrization of SO(3))
        dx_b     %the cartesian velocity of the base (R^3)
        omega_b  %the velocity describing the orientation of the base (SO(3))
        kinematic_chain_selector % list of kinematic chain in the icub
        cur_chain                % current chain that we want to control
    end
	
	methods
		function ro = iCub(obj)
			%% Initialize the mexWholeBodyModel
	        wbm_modelInitialise('icubGazeboSim');
			leftArmInit  = [ -20   30  0.0  45   0.0]';          
            rightArmInit = [ -20   30  0.0  45   0.0]'; 
            torsoInit    = [ -10.0   0.0    0.0]';
			if     obj.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
				% initial conditions for balancing on two feet 
				 leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
				 rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
			elseif   obj.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
			% initial conditions for the robot standing on the left foot
				 leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
				 rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 	 
			elseif   obj.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
			% initial conditions for the robot standing on the right foot
				leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  -0.1]';
				rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]'; 
			end
			qjInit      = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);
			dqjInit     = zeros(obj.ndof,1)
			dx_bInit    = zeros(3,1);
			omega_bInit = zeros(3,1);
			%% Updating the robot position
			wbm_updateState(qjInit,dqjInit,[dx_bInit;omega_bInit]);
			% fixing the world reference frame w.r.t. the foot on ground position
			if obj.feet_on_ground(1) == 1
				[obj.R_b0,obj.x_b0] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
			else  
				[obj.R_b0,obj.x_b0] = wbm_getWorldFrameFromFixedLink('r_sole',qjInit);    
			end
			% define world frame
			wbm_setWorldFrame(R_b0,x_b0,[0 0 -9.81]')	 
		end
	end
	 
end
