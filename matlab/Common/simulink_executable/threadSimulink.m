fullPath = which('find_simulatorIcubSim.m');
simulink_schemes_global = fileparts(fullPath);
cd(simulink_schemes_global)

%inputData has to be saved in common/simulink_executable
load('inputData.mat')

s=' ';
if(strcmp(params.codyco,'old'))
    command_cleaning = ['gnome-terminal -- sh -c "cd' s simulink_schemes_global s '&& ./cleanwholebodytree.sh; bash"'];
else
    command_cleaning = ['gnome-terminal -- sh -c "cd' s simulink_schemes_global s '&& ./cleanwholebody.sh; bash"'];
end

system(command_cleaning)
%system('gnome-terminal -x sh -c "cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./cleanwholebodytree.sh; bash"')

%% TODO add system kill
pause(3);
path_to_pid_clean_wbt = [simulink_schemes_global,'/pid_clean_wbt.txt'];
fileID = fopen(path_to_pid_clean_wbt);
C = textscan(fileID,'%s');
fclose(fileID);
commandkill = strcat('kill -9',{' '},C{1});
system(commandkill{1});

%% TODO pass the name of the scheme from outside
%% this is the scheme used normally for optimization
simulink_scheme_path = [params.path_to_local_simscheme '/' params.name_simulink_schemes];
results = sim(simulink_scheme_path,'SimulationMode','normal');
%% this is the ground truth scheme
%results = sim('torqueBalancing2012b_GT','SimulationMode','normal');
%% with this function i save only the data that i need for computing 
%% the fitness function from the simulink experiment
cd(simulink_schemes_global)
params.messenger.StoreFromSimulink(results);

exit






