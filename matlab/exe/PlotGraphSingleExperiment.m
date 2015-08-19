% compute all the graph for a single experiment
clear variables
close all
clc

name = '2_Jaco1.1_scene1';
starting_point = 1;
total_number_of_experiments = 20;



allpath=which('FindData.m');
fold_path=fileparts(allpath);

for iter = starting_point:total_number_of_experiments
   name_folder = strcat(num2str(iter),'_of_',name);
   disp(name_folder);
   fold_complete_path = strcat(fold_path,'/results/',name,'/',name_folder);
   load(strcat(fold_complete_path,'/data.mat'));
   controller.current_time =[]; % i need to make it empty for restarting simulation
   PlotCmaesResult(fold_complete_path,time_sym_struct,controller,qi,qdi,fixed_step,torque_saturation,name_scenario,time_struct,bestAction,bot1);
   clear time_sym_struct controller qi qdi fixed_step torque_saturation name_scenario time_struct bestAction bot1
end

disp('GENERATION COMPLETED!');