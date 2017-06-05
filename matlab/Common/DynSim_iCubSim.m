%%
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set
%  how many feet are on the ground, decide if activate floating base or not

function [t, q, qd] = DynSim_iCubSim(controller,params)
    
   %% TODO in the config file i have to define the time and the number of dof 
   %% to provide this information outside simulink
    WS = controller.GetWholeSystem();
    %% Updating the robot position and define the world link
    WS.SetWorldFrameiCub(params.qjInit,params.dqjInit,params.dx_bInit,params.omega_bInit,params.root_reference_link);
    WS.ComputeSupportPoly(params);
    %% collecting data from experiments
    time = params.tStart:params.sim_step:params.tEnd;
    
    controller.simulation_iterator     = 1;
    controller.simulation_results.tau  =  zeros(length(time),WS.ndof);
    controller.simulation_results.zmp  =  zeros(length(time),2);
    controller.simulation_results.xCoM =  zeros(length(time),3);
    controller.simulation_results.Cop  =  zeros(length(time),4);
    controller.simulation_results.fc   =  cell(length(time),1);
    
    
    % precompute as time series desidered com and its derivatives
    data_xCoMDes    =  zeros(length(time),3);
    data_dxCoMDes   =  zeros(length(time),3);
    data_ddxCoMDes  =  zeros(length(time),3);
    index = 1;
    for ti=time
        [xCoMDes,dxCoMDes,ddxCoMDes]   = controller.references.GetTraj(1,1,ti);
        data_xCoMDes(index,:)          = xCoMDes;
        data_dxCoMDes(index,:)         = dxCoMDes;
        data_ddxCoMDes(index,:)        = ddxCoMDes;
        index = index + 1;
    end
    ts_xCoMDes    = timeseries(data_xCoMDes,time);
    ts_dxCoMDes   = timeseries(data_dxCoMDes,time);
    ts_ddxCoMDes  = timeseries(data_ddxCoMDes,time);
    %try 
        %% TODO devo cambiare la cartella nel file di configurazione esterno
        options = simset('SrcWorkspace','current');
        sim('torqueBalancing2012b',[],options);
        controller.simulation_iterator     = 1;
        controller.simulation_results.tau  =  torque_sim.Data;
        controller.simulation_results.zmp  =  zmp_sim.Data;
        controller.simulation_results.xCoM =  com_pos_sim.Data;
        q  = q_sim.Data; % row vectors (TODO check if they are in the right order)
        qd = qd_sim.Data;% row vectors
        t  = params.tStart:params.sim_step:params.tEnd;
%     catch err
%         disp('integration error');
%         rethrow(err);
%     end
end










