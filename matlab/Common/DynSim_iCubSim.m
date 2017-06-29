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
    
    timer = 0;
    %% TODO pass this from outside
    max_timer = 100;
    consecutive_fails_counter = 0;
    %% TODO pass this from outside
    max_consecutive_fails_counter = 5;
    
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
    % save all the releavnt data for the thread
    %% commented to test how much variance i have between each simulation
    save('inputData.mat','ts_xCoMDes','ts_dxCoMDes','ts_ddxCoMDes','params');
    %% execution of the process and check of success 
    system('gnome-terminal -x sh -c " cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./save_pid.sh &&   ./matlab_link  -nodesktop  -r threadSimulink; bash"');
    % waiting for the thread completion
    fid=fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/simulationResults.mat');
    while(fid == -1)
        pause(5)
        fid=fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/simulationResults.mat');
        timer = timer + 5;
        % if the process get stuck i can close it and open a new one 
        if (timer>=max_timer)
            % close the bash window of the matlab process
            fileID = fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/pid.txt');
            C = textscan(fileID,'%s');
            fclose(fileID);
            commandkill = strcat('kill -9',{' '},C{1});
            system(commandkill{1});
            system('gz world -r');
            pause(3);
            % create a new process with the same value
            system('gnome-terminal -x sh -c " cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./save_pid.sh && ./matlab_link -nodesktop -r threadSimulink; bash"');
            timer = 0;
            consecutive_fails_counter = consecutive_fails_counter + 1;
            disp('thread got stuck i had to restart it')
            if(consecutive_fails_counter >= max_consecutive_fails_counter)
                        system('pkill -f wholeBodyDynamicsTree ')
                        pause(3)
                        system('pkill -f gazebo')
                        pause(3)
                        system('pkill -f yarpserver')
                        pause(3)
                        clear simulator
                        pause(3)
                        system('gnome-terminal -x sh -c "yarpserver; bash"');
                        pause(3)
                        system('gnome-terminal -x sh -c "cd ~/git/learnOptimWBC/matlab/TestResults/scenarios/ && gazebo -slibgazebo_yarp_clock.so sit_icub_to_optimize_0_1.world; bash"');
                        pause(3)
                        system('gnome-terminal -x sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"');
                        pause(3)
                        disp('too many consecutive fails i restart all the programs')
                        consecutive_fails_counter = 0;
            end
            
        end
        
    end
    %% copy the results from the thread and close the thread
    load('simulationResults.mat');
    % remove the file with all the data inside
    delete ~/git/learnOptimWBC/matlab/Common/TB_StandUp/simulationResults.mat
    % close the bash window of the matlab process
    fileID = fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/pid.txt');
    C = textscan(fileID,'%s');
    fclose(fileID);
    commandkill = strcat('kill -9',{' '},C{1});
     % kill windows hosting the  matlab thread
    system(commandkill{1});
    % clean yarp
    [xx,yy]=system('yarp clean  --timeout 1.0');
    %% save data 
    controller.simulation_iterator     = 1;
    controller.simulation_results.tau  =  torque_sim.Data;
    controller.simulation_results.zmp  =  zmp_sim.Data;
    number_of_dims = ndims(com_pos_sim.Data);
    if(number_of_dims>2)
        app_mat = squeeze(com_pos_sim.Data);
        [row,col] = size(app_mat);
        if(row<col)
            app_mat = app_mat';
        end
        controller.simulation_results.xCoM =  app_mat;
    else
        controller.simulation_results.xCoM =  com_pos_sim.Data;
    end
    controller.simulation_results.LsoleWrench =  left_leg_wrench_sim;
    controller.simulation_results.RsoleWrench =  right_leg_wrench_sim; 
    q  = q_sim.Data; % row vectors (TODO check if they are in the right order)
    qd = qd_sim.Data;% row vectors
    t  = params.tStart:params.sim_step:params.tEnd;
end










