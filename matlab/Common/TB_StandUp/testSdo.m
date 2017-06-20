%% test simulator 




% finish_simulation = 0;
% %options = simset('SrcWorkspace','current');
% exec_time  = 4.5;
% reset_time = 4.5;
% delay_time = 3;
% total_time = 0;
% 
% total_reset_time = 0;
% 
% for ijk = 1:1

%     %% exec block        
%     total_time = total_time + exec_time;
%     set_param('torqueBalancing2012b/PauseTrigger','Value', num2str(total_time));
%     set_param('torqueBalancing2012b/ResetTrigger','Value', num2str(0));
%     set_param('torqueBalancing2012b', 'SimulationCommand', 'continue');
%     while(~finish_simulation)
%         pause(exec_time + delay_time);
%     end
%     finish_simulation = 0;
%    
%     %% reset block 
%     total_time = total_time + reset_time;
%     total_reset_time = total_reset_time + reset_time;
%     set_param('torqueBalancing2012b/PauseTrigger','Value', num2str(total_time));
%     set_param('torqueBalancing2012b/ResetTrigger','Value', num2str(1));
%     set_param('ResetScheme/PauseTrigger','Value', num2str(total_reset_time));
%     set_param('torqueBalancing2012b', 'SimulationCommand', 'continue');
%     set_param('ResetScheme', 'SimulationCommand', 'continue');
%     
%     while(~finish_simulation)
%         pause(reset_time);
%     end
%      finish_simulation = 0;
%      
%     system('gz world -o');
%    
%     total_time = total_time + reset_time;
%     total_reset_time = total_reset_time + reset_time;
%     set_param('torqueBalancing2012b/PauseTrigger','Value', num2str(total_time));
%     set_param('ResetScheme/PauseTrigger','Value', num2str(total_reset_time));
%     set_param('torqueBalancing2012b', 'SimulationCommand', 'continue');
%      set_param('ResetScheme', 'SimulationCommand', 'continue');
%     while(~finish_simulation)
%         pause(reset_time);
%     end
%     finish_simulation = 0;
%     ijk
% end

clear variables 
close all 
clc
%simulator = sdo.SimulationTest('torqueBalancing2012b');
reset_turns = 1;
total_it    = 5;
timer = 0;
max_time = 100;
consecutive_fails_counter = 0;
max_consecutive_fails_counter = 5;


% create symbolic link of matlab executable in TB_standUp 
system('cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && sh create_symbolic_link.sh')
% check if yarpserver is running
[a,pid]=system('pgrep yarpserver');
if(strcmp('',pid))
    system('gnome-terminal -x sh -c "yarpserver; bash"')
    pause(15)
else
    disp('yarpserver already running')
end
% check if gazebo is running
[a,pid]=system('pgrep gazebo');
if(strcmp('',pid))
    system('gnome-terminal -x sh -c "cd ~/git/learnOptimWBC/matlab/TestResults/scenarios/ && gazebo -slibgazebo_yarp_clock.so sit_icub_to_optimize_0_1.world; bash"');
    pause(15)
else
    disp('gazebo already running')
end
% check if gazebo is running
[a,pid]=system('pgrep wholeBody');
if(strcmp('',pid))
    system('gnome-terminal -x sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"')
    pause(15)
else
    disp('wholeBodyDynamicsTree already running')
end


for i = 1:total_it
  
    
    %sim(simulator);
    system('gnome-terminal -x sh -c " cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./save_pid.sh &&   ./matlab_link -nodesktop -r test_thread; bash"');
    %system('gnome-terminal -x sh -c "cd ~/MATLAB/r2013a/bin && ./matlab  -nodesktop -r test_thread; bash"');
%     if(counter>=reset_turns)
%         system('pkill -f wholeBodyDynamicsTree ')
%         pause(3)
%         system('pkill -f gazebo')
%         pause(3)
%         system('pkill -f yarpserver')
%         pause(3)
%         clear simulator
%         pause(3)
%         system('gnome-terminal -x sh -c "yarpserver; bash"');
%         pause(3)
%         system('gnome-terminal -x sh -c "cd ~/git/learnOptimWBC/matlab/TestResults/scenarios/ && gazebo -slibgazebo_yarp_clock.so sit_icub_to_optimize_0_1.world; bash"');
%         pause(3)
%         system('gnome-terminal -x sh -c "wholeBodyDynamicsTree --autoconnect --robot icubSim; bash"');
%         pause(3)
%         simulator = sdo.SimulationTest('torqueBalancing2012b');
%         pause(15)
%         counter = 1;
%     else
    fid=fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/test_file.dat');
    while(fid == -1)
        pause(5)
        fid=fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/test_file.dat');
        timer = timer + 5;
        % if the process get stuck i can close it and open a new one 
        if (timer>=max_time)
            % close the bash window of the matlab process
            fileID = fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/pid.txt');
            C = textscan(fileID,'%s');
            fclose(fileID);
            commandkill = strcat('kill -9',{' '},C{1});
            system(commandkill{1});
            system('gz world -r');
            pause(3);
            % create a new process with the same value
            system('gnome-terminal -x sh -c " cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./save_pid.sh && cd ~/MATLAB/r2013a/bin/ && ./matlab -nodesktop -r test_thread; bash"');
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
                        disp('too many consecutive fails i restart all the progrmas')
                        consecutive_fails_counter = 0;
            end
            
        end
        
    end
    timer = 0;
    consecutive_fails_counter = 0;
    % remove the file with all the data inside
    delete ~/git/learnOptimWBC/matlab/Common/TB_StandUp/test_file.dat
    % close the bash window of the matlab process
    fileID = fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/pid.txt');
    C = textscan(fileID,'%s');
    fclose(fileID);
    commandkill = strcat('kill -9',{' '},C{1});
    % kill windows hosting the  matlab thread
    system(commandkill{1});
    % reset gazebo world
    system('gz world -r');
    pause(3);
    
%    end
    
    i 
    %counter = counter + 1;
    
end
