%%
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set
%  how many feet are on the ground, decide if activate floating base or not

function [t, q, qd] = DynSim_iCubSim(controller,params)
    
   %% TODO in the config file i have to define the time and the number of dof 
   %% to provide this information outside simulink
    WS = controller.GetWholeSystem(); 
    time = params.tStart:params.sim_step:params.tEnd;
    
    controller.simulation_iterator     = 1;
    controller.simulation_results.tau  =  zeros(length(time),WS.ndof);
    controller.simulation_results.zmp  =  zeros(length(time),2);
    controller.simulation_results.xCoM =  zeros(length(time),3);
    controller.simulation_results.Cop  =  zeros(length(time),4);
    controller.simulation_results.fc   =  cell(length(time),1);
    
     
    %try 
        %% TODO devo cambiare la cartella nel file di configurazione esterno
        options = simset('SrcWorkspace','current');
        sim('torqueBalancing2012b',[],options);
        controller.simulation_iterator     = 1;
        controller.simulation_results.tau  =  torque_sim;
        controller.simulation_results.zmp  =  zmp_sim;
        controller.simulation_results.xCoM =  com_pos_sim;
        controller.simulation_results.Cop  =  zeros(length(time),4);
        controller.simulation_results.fc   =  cell(length(time),1);
        q  = q_sim; % row vectors (TODO check if they are in the right order)
        qd = qd_sim;% row vectors
        t  = params.tStart:params.sim_step:params.tEnd;
%     catch err
%         disp('integration error');
%         rethrow(err);
%     end
end










