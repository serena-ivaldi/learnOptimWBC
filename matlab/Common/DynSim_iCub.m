%%
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set
%  how many feet are on the ground, decide if activate floating base or not

% #TODO substitute icub with controller.subchains
function [t, q, qd] = DynSim_iCub(controller,params)
    WS = controller.GetWholeSystem();
    %% Updating the robot position and define the world link
    WS.SetWorldFrameiCub(params.qjInit,params.dqjInit,params.dx_bInit,params.omega_bInit,params.root_reference_link);
    WS.ComputeSupportPoly(params);
    [~,T_b,~,~] = WS.GetState();

    params.chiInit = [T_b; params.qjInit; WS.state.dx_b; WS.state.w_omega_b; params.dqjInit];
    %% Integrate forward dynamics
    if params.demo_movements == 0
        options = odeset('RelTol',1e-3,'AbsTol', 1e-4);
    else
        options = odeset('RelTol',1e-6,'AbsTol',1e-6);
    end

    params.lfoot_ini = wbm_forwardKinematics('l_sole');
    params.rfoot_ini = wbm_forwardKinematics('r_sole');
    params.lu_leg_ini = wbm_forwardKinematics('l_upper_leg');
    params.ru_leg_ini = wbm_forwardKinematics('r_upper_leg');
    
    
    %% initilization for controller visual
     controller.visual_param.t    =  [];
     controller.visual_param.tau  =  [];
     controller.visual_param.qj   =  [];
     controller.visual_param.zmp  =  [];
     controller.visual_param.xCoM =  [];

    forwardDynFunc  = @(t,chi)DynSim_iCubForwardDynamics(t,chi,controller,params);

    try                        
        [t,chi,visual_param] = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);
        q = chi(:,1:7+WS.ndof);
        qd = chi(:,8+WS.ndof:end);
        %delete(params.wait)
    catch err
        disp('integration error');
        rethrow(err);
    end
end










