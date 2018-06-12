%%
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set
%  how many feet are on the ground, decide if activate floating base or not

function [t, q, qd,failed_flag] = DynSim_iCubSim(controller,params)
    
    cd(params.simulink_schemes_global)

    %% with this function, I store all the data necessary for the execution of the experiments
    params.messenger.Pack(controller,params)
    
    %% launch the experiment and check if the process hangs 
    timer = 0;
    consecutive_fails_counter = 0;
    failed_flag = false;

    %% ---
    % for spacing strings
    s=' ';
    %% execution of the process and check of success 
    run_command = ['gnome-terminal -- sh -c " cd' s params.simulink_schemes_global s '&& ./save_pid.sh && ./matlab_link -nodesktop -r threadSimulink; bash"'];
    system(run_command);
    
    % waiting for the thread completion
    simulation_result_path = [params.simulink_schemes_global '/simulationResults.mat'];
    fid=fopen(simulation_result_path);
    while(fid == -1)
        pause(5)
        fid=fopen(simulation_result_path);
        timer = timer + 5;
        % if the process gets stuck, I can close it and open a new one 
        if (timer >= params.max_timer)
            % close the bash window of the matlab process
            pid_path = [params.simulink_schemes_global '/pid.txt'];
            fileID = fopen(pid_path);
            C = textscan(fileID,'%s');
            fclose(fileID);
            commandkill = strcat('kill -9',{' '},C{1});
            system(commandkill{1});
            setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
            system('gz world -r');
            setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
            pause(3);
            % create a new process with the same value
            run_command = ['gnome-terminal -- sh -c " cd' s params.simulink_schemes_global s '&& ./save_pid.sh && ./matlab_link -nodesktop -r threadSimulink; bash"'];
            system(run_command);
            timer = 0;
            consecutive_fails_counter = consecutive_fails_counter + 1;
            disp('thread got stuck, i had to restart it')
            if(consecutive_fails_counter >= params.max_consecutive_fails_counter)
                        if(strcmp(params.codyco,'old'))
                            system('pkill -f wholeBodyDynamicsTree ')
                        else
                            %A human may need to press ctrl-c 4 times
                            system('pkill -f yarprobotinterf'); 
                            system('pkill -f yarprobotinterf');
                            system('pkill -f yarprobotinterf');
                            system('pkill -f yarprobotinterf');
                        end
                        pause(3)
                        system('pkill -f gazebo')
                        pause(3)
                        system('pkill -f yarpserver')
                        pause(3)
                        clear simulator
                        pause(3)
                        setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
                        system('gnome-terminal -- sh -c "yarpserver; bash"');
                        setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
                        pause(3)
                        scenario_path = which('FindData.m');
                        scenario_path = fileparts(scenario_path);
                        scenario_path = strcat(scenario_path,'/scenarios');
                        command_gazebo = ['gnome-terminal -- sh -c "cd' s scenario_path s '&& gazebo -slibgazebo_yarp_clock.so'];
                        command_gazebo = [command_gazebo,s,params.scenario_name '; bash"'];
                        setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
                        system(command_gazebo)
                        setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
                        pause(3)
                        % this is a temporary switch for the old codyco branch; it will be deleted in the future
                        if(strcmp(params.codyco,'old'))
                            setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
                            system('gnome-terminal -- sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"');
                            setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
                        else
                            setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
                            system('gnome-terminal -- sh -c "yarprobotinterface --config launch-wholebodydynamics.xml; bash"');
                            setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
                        end
                        pause(3)
                        disp('too many consecutive fails; I restart all the programs')
                        consecutive_fails_counter = 0;
            end
            
        end
        
    end
    %% copy the results from the thread 
    try
        %% with this function I collect the results from the simulink experiment
        %% and store them in controller.simulation_results and [q,qd,t]
        [q,qd,t]=params.messenger.Unpack(controller,params);        
    catch
        disp('the results from the simulation are corrupted or not present. Repeat the simulation with the same parameters')
        failed_flag = true;
    end
    
    if size(q,1) <= 1
        % if there was no error, but the simulation did not run 
        % due to early termination that occurred right at initial time,
        % gazebo was unable to replace the robot in its original position,
        % therefore gazebo and yarprobotinterface need to be restarted
        % and the simulation repeated
        failed_flag = true;
        %% DEBUG
        disp('Something went wrong; restarting gazebo and repeating the simulation with the same parameters');
        if(strcmp(params.codyco,'old'))
            system('pkill -f wholeBodyDynamicsTree ');
        else
            %A human may need to press ctrl-c 4 times
            system('pkill -f yarprobotinterf');
            system('pkill -f yarprobotinterf');
            system('pkill -f yarprobotinterf');
            system('pkill -f yarprobotinterf');
        end
        pause(3)
        system('pkill -f gazebo');
        pause(3)
        setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
        [xx,yy] = system('yarp clean  --timeout 1.0');
        setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
        scenario_path = which('FindData.m');
        scenario_path = fileparts(scenario_path);
        scenario_path = strcat(scenario_path,'/scenarios');
        command_gazebo = ['gnome-terminal -- sh -c "cd' s scenario_path s '&& gazebo -slibgazebo_yarp_clock.so'];
        command_gazebo = [command_gazebo,s,params.scenario_name '; bash"'];        
        setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
        system(command_gazebo);
        setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
        pause(3)
        % this is a temporary switch for the old codyco branch; it will be deleted in the future
        if(strcmp(params.codyco,'old'))
            setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
            system('gnome-terminal -- sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"');
            setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
        else
            setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
            system('gnome-terminal -- sh -c "yarprobotinterface --config launch-wholebodydynamics.xml; bash"');
            setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
        end
        pause(3)
    end
    
    % if I have a fail, return empty data 
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
    % kill windows hosting the matlab thread
    system(commandkill{1});
    % clean yarp
    setenv('LD_LIBRARY_PATH', params.new_matlab_LD_LIBRARY_PATH);
    [xx,yy]=system('yarp clean  --timeout 1.0');
    setenv('LD_LIBRARY_PATH', params.matlab_LD_LIBRARY_PATH);
end










