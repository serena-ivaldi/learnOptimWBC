function [simulink_schemes_global,local_path]=SimulinkInitializationExperiment(name_simulink_model,scenario_name,codyco)
    s = ' ';
    %% change folder (move to the folder with the simulink scheme)
    fullPath = which('find_simulatorIcubSim.m');
    simulink_schemes_global = fileparts(fullPath);
    local_path = strcat(simulink_schemes_global,'/',name_simulink_model);
    cd(simulink_schemes_global)
    
    path_to_simulation_results = strcat('./',name_simulink_model,'/simulationResults.mat');
    delete(path_to_simulation_results);
    % here i open all the program that are necessary for the simulation
    % execution and i create a symbolic link to matlab
    % create symbolic link of matlab executable in TB_standUp
    
    %system('cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && sh create_symbolic_link.sh')
    matlab_path = matlabroot;
    matlab_path = [matlab_path '/bin/matlab'];
    command_symbolic_link = ['ln -s' s matlab_path s 'matlab_link && chmod 777 matlab_link'];
    system(command_symbolic_link)
    % check if yarpserver is running
    [a,pid]=system('pgrep yarpserver');
    if(strcmp('',pid))
        system('gnome-terminal -- sh -c "yarpserver; bash"')
        pause(15)
    else
        disp('yarpserver already running')
    end
    % check if gazebo is running
    [a,pid]=system('pgrep gazebo');
    if(strcmp('',pid))
        scenario_path = which('FindData.m');
        scenario_path = fileparts(scenario_path);
        scenario_path = strcat(scenario_path,'/scenarios');
        command_gazebo = ['gnome-terminal -- sh -c "cd' s scenario_path s '&& gazebo -slibgazebo_yarp_clock.so'];
        command_gazebo = [command_gazebo,s,scenario_name '; bash"'];
        system(command_gazebo)
        pause(15)
    else
        disp('gazebo already running')
    end
      % check if gazebo is running
    if(strcmp(codyco,'old'))
        [a,pid]=system('pgrep wholeBody');
    else
        [a,pid]=system('pgrep yarprobotinterf');
    end
    
    if(strcmp('',pid))
        % this is a temporary switch. the old codyco branch it will be
        % deleted in the future
        if(strcmp(codyco,'old'))
            system('gnome-terminal -- sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"')
        else
            system('gnome-terminal -- sh -c "yarprobotinterface --config launch-wholebodydynamics.xml; bash"')
        end
        pause(15)
    else
        disp('wholeBodyDynamicsTree already running')
    end

    % reset the robot in the starting position
    system('gz world -r');

    %% remove the results file from the the TBstandup folder to manage the case where i did not cancel this file properly
    % remove the file with all the data inside
    
end