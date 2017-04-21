%%
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set
%  how many feet are on the ground, decide if activate floating base or not

% #TODO substitute icub with controller.subchains
function [t, q, qd] = FakeDynSim_iCub(controller,params)
    WS = controller.GetWholeSystem();

    %% Integrate forward dynamics
    if params.demo_movements == 0
        options = odeset('RelTol',1e-3,'AbsTol', 1e-4);
    else
        options = odeset('RelTol',1e-6,'AbsTol',1e-6);
    end    
    
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
     
    
    %try 
        index = 1;
        for ti = time
            q(index,:) =  zeros(1,7+WS.ndof);
            qd(index,:) = zeros(1,WS.ndof);
            index = index + 1;
        end
        
        t = controller.simulation_results.t;
        %delete(params.wait)
%     catch err
%         disp('integration error');
%         rethrow(err);
%     end
end










