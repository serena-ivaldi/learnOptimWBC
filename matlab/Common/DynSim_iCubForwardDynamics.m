function [dchi,fitness_param]=DynSim_iCubForwardDynamics(t,chi,controller,param)
    %% Config parameters
    import WBM.utilities.dquat;
    % i get the pointer to the whole system
    if isempty(controller.current_time)
        controller.current_time = tic;
    end

    icub = controller.GetWholeSystem();
    ndof = icub.ndof;
    disp(t)

    %% Extraction of state
    [state,x_b,qt_b,w_R_b,base_pose,q,dx_b,w_omega_b,qd,Nu]=icub.State(chi);

    %% Update state (in the robot model) (it is necessary when i want to recompute offline data from the current rollout)
    wbm_setWorldFrame_v1(w_R_b,x_b,[0 0 -9.81]');
    wbm_updateState_v1(q,qd,[dx_b;w_omega_b]);

    %% dynamics
    [dynamic,M,h,g,H,C_nu,JCoM,dJCoM_nu,JH,dJH_nu] = icub.Dynamics();
    
   
    %% correction to ensure the mass matrix to be positive definite. It is used
    %% only when the system is integrated using a fixed step integrator,
    %% otherwise it won't converge to a solution.
    M(7:end,7:end)               = M(7:end,7:end) + param.massCorr.*eye(ndof);
    dynamic.M(7:end,7:end)       = dynamic.M(7:end,7:end) + param.massCorr.*eye(ndof);
    icub.dynamic.M(7:end,7:end)  = icub.dynamic.M(7:end,7:end) + param.massCorr.*eye(ndof);
    
    %% compute com position (TODO (provisory) i need to reorder how the kinematics is computed)
    poseCoM  = wbm_forwardKinematics_v1(icub.state.w_R_b,icub.state.x_b,q,'com');
    xCoM     = poseCoM(1:3);
    %% update contact state (i  suppose that i start to move after 0.1 seconds)
    if(t>0.1)
       param.feet_on_ground(3) = 0;
       param.feet_on_ground(4) = 0; 
       param.numContacts = sum(param.feet_on_ground);
       param.contact_sym.state(3) = 0;
       param.contact_sym.state(4) = 0;
       param.contact_sym.UpdateContact();
    end
  
    %% TODO
    %% contact jacobian
    [jacobian_contact,Jc_sym,dJcNu_sym] = icub.ContactJacobians(param,param.contact_sym);
    %% Control torques calculation
    tau = controller.Policy(t,q,qd,[],jacobian_contact,param); 
    %% torque saturation
    tau(tau>param.torque_saturation)  = param.torque_saturation;
    tau(tau<-param.torque_saturation) = -param.torque_saturation;

    %% contact dynamic simulation    
    [tauContact,fc]=DynSim_iCubContacts(ndof,state,dynamic,Jc_sym,dJcNu_sym,param,tau);
    %% compute zmp
    [zmp ,Cop]=DynSim_iCubZmp(fc,param,x_b,w_R_b,q); 
    %% State derivative computation
    % Need to calculate the quaternions derivative
    b_omega_w = transpose(w_R_b)*w_omega_b; %% TODO floating base flag required (parameter of the simulator)
    dqt_b   = dquat(qt_b,b_omega_w);   %% TODO floating base flag required (parameter of the simulator)
    Nu      = [dx_b;dqt_b;qd];
    dNu     = M\(tauContact + [zeros(6,1); tau]-h);
    dchi    = [Nu;dNu];

    %% TO PUT BACK in this way i remove the test input that are too long 
%     if toc(controller.current_time) > param.maxtime;
%        controller.current_time = [];
%        disp('Stopped. Taking too long.')
%        error('Stopped. Taking too long.')
%     end
    %% Visualization
    % These are the variables that can be plotted by the visualizer.m
    % function
    if(param.integrateWithFixedStep)
        index                                       = controller.simulation_iterator;
        controller.simulation_results.t(index)      = t;
        controller.simulation_results.tau(index,:)  = tau';
        controller.simulation_results.q(index,:)    = q';
        controller.simulation_results.zmp(index,:)  = zmp;
        controller.simulation_results.xCoM(index,:) = xCoM';
        controller.simulation_results.Cop(index,:)  = Cop;
        controller.simulation_results.fc{end + 1}   = fc';
        controller.simulation_iterator              = controller.simulation_iterator + 1;
    else
        fitness_param.Cop  =  Cop;
        fitness_param.fc   =  fc';
        fitness_param.t    =  t;
        fitness_param.tau  =  tau';
        fitness_param.q    =  q';
        fitness_param.zmp  =  zmp;
        fitness_param.xCoM =  xCoM';
    end
end