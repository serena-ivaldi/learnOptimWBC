%% global variable
name_file = {'g06_g07_g09_f240_f241_HB_','RP_humanoid_bench_lbrsimple_RP_humanoid_bench_lbrsimple_more_constrained_'};
metric_whom_compute_mean_and_variance = {'metric1','metric2','metric3';'metric7','metric2','metric3'}; % at each row correspond different set of experiments
experiments_selector = {[1 2] ; [2 3 6]};
%% path to dat file to open
%%  SAVE PATH
 % parameter
 folder = 'benckmark';
 %% IMP!!! this vector represents the order in which the istogram related to the i-th method is showed in the graph
 subfolder = {'(1+1)CMAES-vanilla','CMAES-vanilla','CMAES-adaptive','fmincon-fmincon'};
 allpath=which('FindData.m');
 local_path=fileparts(allpath);
 
 
 %% LOAD DATA
 for j = 1:length(name_file)
     for i=1:length(subfolder)
        cur_mat = strcat(local_path,'/',folder,'/',subfolder{i},'/',name_file,'.mat');
        for k = 1 : size(metric_whom_compute_mean_and_variance,2)
            load(cur_mat,metric_whom_compute_mean_and_variance{j,k});
             store_data{i} = eval(metric_whom_compute_mean_and_variance{j,k});
        end
     end
     
     
     
 end