classdef iCub < handle
 
	properties     
        %% Structural Parameters
        ndof   % dependant on the model used for the simulaztion
        list_of_kin_chain  %string matching URDF name of the link (frame)
        dim_of_kin_chain 
        access_index; % vector of index used for accesing the infromation stored in the structural parameters
        %% Computational parameters (used as placeholder during the computation of the dynamics of the robot)
        % replace all of this value with wbm_getState
        %x_b      %the cartesian position of the base (R^3)
        %qt_b     %the quaternion describing the orientation of the base (global parametrization of SO(3))
        %dx_b     %the cartesian velocity of the base (R^3)
        %omega_b  %the velocity describing the orientation of the base (SO(3))
        kinematic_chain_selector % list of kinematic chain in the icub
        cur_chain                % current chain that we want to control
    end
	
	methods
		function ro = iCub(obj)
			% Initialize the mexWholeBodyModel
	        wbm_modelInitialise('icubGazeboSim');	
            obj.list_of_kin_chain = {'r_sole','l_sole','com','right_arm','left_arm'}; %string matching URDF name of the link (frame)
            obj.dim_of_kin_chain  = {6,6,3,5,5};
            obj.ndof = 25;
        end
        
        function SetWorldFrameiCub(obj,qjInit,reference_link)
            %% Updating the robot position
			wbm_updateState(qjInit,dqjInit,[dx_bInit;omega_bInit]);
			% fixing the world reference frame w.r.t. the foot on ground position
			[R_b0,x_b0] = wbm_getWorldFrameFromFixedLink(reference_link,qjInit);
			% define world frame
			wbm_setWorldFrame(R_b0,x_b0,[0 0 -9.81]')
        end
        
        function [qj,xTb,qjDot,vb] = GetBaseState(obj)
            [qj,xTb,qjDot,vb]=wbm_getState();
        end
        
	end
	 
end
