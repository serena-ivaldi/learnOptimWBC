cd  ~/git/learnOptimWBC/matlab/Common/TB_StandUp/


simulator = sdo.SimulationTest('torqueBalancing2012b');
sim(simulator);

pippo = 3;
save('test_file.dat','pippo')

exit