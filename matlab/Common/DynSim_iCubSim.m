%%
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set
%  how many feet are on the ground, decide if activate floating base or not

function [t, q, qd,failed_flag] = DynSim_iCubSim(controller,params)
    
    cd(params.simulink_schemes_global)


    %% with this function i store all the data that are necessary for the execution of the experiments
    params.messenger.Pack(controller,params)
    
    %% launch the experiment and check if the process hang 
    timer = 0;
    consecutive_fails_counter = 0;
    failed_flag = false;
    %% TODO pass this from outside
    max_timer = 100;
    max_consecutive_fails_counter = 5;
    %% ---
    % for spacing strings
    s=' ';
    %% execution of the process and check of success 
    run_command = ['gnome-terminal -- sh -c " cd' s params.simulink_schemes_global s '&& ./save_pid.sh && ./matlab_link -nodesktop -r threadSimulink; bash"'];
    system(run_command);
    %system('gnome-terminal -x sh -c " cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./save_pid.sh &&   ./matlab_link  -nodesktop  -r threadSimulink; bash"');
    % waiting for the thread completion
    simulation_result_path = [params.simulink_schemes_global '/simulationResults.mat'];
    fid=fopen(simulation_result_path);
    while(fid == -1)
        pause(5)
        fid=fopen(simulation_result_path);
        timer = timer + 5;
        % if the process get stuck i can close it and open a new one 
        if (timer>=max_timer)
            % close the bash window of the matlab process
            pid_path = [params.simulink_schemes_global '/pid.txt'];
            fileID = fopen(pid_path);
            C = textscan(fileID,'%s');
            fclose(fileID);
            commandkill = strcat('kill -9',{' '},C{1});
            system(commandkill{1});
            system('gz world -r');
            pause(3);
            % create a new process with the same value
            run_command = ['gnome-terminal -- sh -c " cd' s params.simulink_schemes_global s '&& ./save_pid.sh && ./matlab_link -nodesktop -r threadSimulink; bash"'];
            system(run_command);
            timer = 0;
            consecutive_fails_counter = consecutive_fails_counter + 1;
            disp('thread got stuck i had to restart it')
            if(consecutive_fails_counter >= max_consecutive_fails_counter)
                        if(strcmp(params.codyco,'old'))
                            system('pkill -f wholeBodyDynamicsTree ')
                        else
                            system('pkill -f yarprobotinterf')
                        end
                        pause(3)
                        system('pkill -f gazebo')
                        pause(3)
                        system('pkill -f yarpserver')
                        pause(3)
                        clear simulator
                        pause(3)
                        system('gnome-terminal -- sh -c "yarpserver; bash"');
                        pause(3)
                        scenario_path = which('FindData.m');
                        scenario_path = fileparts(scenario_path);
                        scenario_path = strcat(scenario_path,'/scenarios');
                        command_gazebo = ['gnome-terminal -- sh -c "cd' s scenario_path s '&& gazebo -slibgazebo_yarp_clock.so'];
                        command_gazebo = [command_gazebo,s,params.scenario_name '; bash"'];
                        system(command_gazebo)
                        pause(3)
                        % this is a temporary switch. the old codyco branch it will be
                        % deleted in the future
                        if(strcmp(params.codyco,'old'))
                            system('gnome-terminal -- sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"');
                        else
                            system('gnome-terminal -- sh -c "yarprobotinterface --config launch-wholebodydynamics.xml; bash"')
                        end
                        pause(3)
                        disp('too many consecutive fails i restart all the programs')
                        consecutive_fails_counter = 0;
            end
            
        end
        
    end
    %% copy the results from the thread 
    try
        %% with this function i collect the results from the simulink experiment 
        %% and i store them in  controller.simulation_results and [q,qd,t]
        [q,qd,t]=params.messenger.Unpack(controller,params);    
    catch 
        disp('the results from the simulation are corrupted or not present. Repeat the simulation with the same parameters')
        failed_flag = true;
    end
    % if i have a fail a return empty data 
    if(failed_flag)
       
        q  = [];
        qd = [];
        t  = [];
    end
    
    %% cleaning files and processes 
    % remove the file with all the data inside
    delete(simulation_result_path)
    %inputData has to be saved in common/simulink_executable
    %delete('inputData.mat')
    % close the bash window of the matlab process
    pid_path = [params.simulink_schemes_global '/pid.txt'];
    fileID = fopen(pid_path);
    C = textscan(fileID,'%s');
    fclose(fileID);
    commandkill = strcat('kill -9',{' '},C{1});
     % kill windows hosting the  matlab thread
    system(commandkill{1});
    % clean yarp
    [xx,yy]=system('yarp clean  --timeout 1.0');
    
end










