cd  ~/git/learnOptimWBC/matlab/Common/TB_StandUp/
system('gnome-terminal -x sh -c "cd ~/git/learnOptimWBC/matlab/Common/TB_StandUp && ./cleanwholebodytree.sh; bash"')

%% TODO add system kill
pause(3);
fileID = fopen('~/git/learnOptimWBC/matlab/Common/TB_StandUp/pid_clean_wbt.txt');
C = textscan(fileID,'%s');
fclose(fileID);
commandkill = strcat('kill -9',{' '},C{1});
system(commandkill{1});
load('inputData.mat')
%simulator = sdo.SimulationTest('torqueBalancing2012b');
%results = sim(simulator);
results = sim('torqueBalancing2012b','SimulationMode','normal');
%options = simset('SrcWorkspace','current');
%sim('torqueBalancing2012b',[],options);
torque_sim           = results.get('torque_sim');
zmp_sim              = results.get('zmp_sim');
com_pos_sim          = results.get('com_pos_sim');
q_sim                = results.get('q_sim');
qd_sim               = results.get('qd_sim');
left_leg_wrench_sim  = results.get('left_leg_wrench_sim');
right_leg_wrench_sim = results.get('right_leg_wrench_sim');


save('simulationResults.mat','torque_sim','zmp_sim','com_pos_sim','q_sim','qd_sim','left_leg_wrench_sim','right_leg_wrench_sim');
exit