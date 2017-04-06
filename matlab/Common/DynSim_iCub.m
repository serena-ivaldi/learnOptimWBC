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
    
    
    %% initilization and at the same time reset for controller simulation_results
    
     time = params.tStart:params.sim_step:params.tEnd;
     
     if(params.integrateWithFixedStep)
         controller.simulation_iterator     = 1;
         controller.simulation_results.t    =  zeros(length(time),1);
         controller.simulation_results.tau  =  zeros(length(time),WS.ndof);
         controller.simulation_results.q    =  zeros(length(time),WS.ndof);
         controller.simulation_results.zmp  =  zeros(length(time),2);
         controller.simulation_results.xCoM =  zeros(length(time),3);
         controller.simulation_results.Cop  =  zeros(length(time),4);
         controller.simulation_results.fc   =  cell(length(time),1);
     end
     
    forwardDynFunc  = @(t,chi)DynSim_iCubForwardDynamics(t,chi,controller,params);

    try 
        
        if(params.integrateWithFixedStep)
            chi = Ode1(forwardDynFunc,time,params.chiInit); 
            t = controller.simulation_results.t;
        else
            [t,chi] = ode15s(forwardDynFunc,time,params.chiInit,options);
        end
        q = chi(:,1:7+WS.ndof);
        qd = chi(:,8+WS.ndof:end);
        %delete(params.wait)
    catch err
        disp('integration error');
        rethrow(err);
    end
end










