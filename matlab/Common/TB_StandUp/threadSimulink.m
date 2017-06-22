cd  ~/git/learnOptimWBC/matlab/Common/TB_StandUp/

load('inputData.mat')
%simulator = sdo.SimulationTest('torqueBalancing2012b');
%results = sim(simulator);
results = sim('torqueBalancing2012b','SimulationMode','normal');
%options = simset('SrcWorkspace','current');
%sim('torqueBalancing2012b',[],options);
torque_sim  = results.get('torque_sim');
zmp_sim     = results.get('zmp_sim');
com_pos_sim = results.get('com_pos_sim');
q_sim       = results.get('q_sim');
qd_sim       = results.get('qd_sim');
save('simulationResults.mat','torque_sim','zmp_sim','com_pos_sim','q_sim','qd_sim');
exit