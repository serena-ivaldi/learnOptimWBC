cd  ~/git/learnOptimWBC/matlab/Common/TB_StandUp/

load('inputData.mat')
simulator = sdo.SimulationTest('torqueBalancing2012b');
sim(simulator);
save('simulationResults.mat','torque_sim','zmp_sim','com_pos_sim','q_sim','qd_sim');
exit