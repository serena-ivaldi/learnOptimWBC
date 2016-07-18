% all the metric that has singular value are going to be organized in
% column

clear all 
close all
clc
%%  Load Path
% parameter
folder = 'benckmark';
% name fo the method
last_subfolder = {'(1+1)CMAES-vanilla','CMAES-vanilla'};
% flag to say if it is or not a robotic experiment or a function benchmark
robotic_flag_per_experiment = [1 1];
% name of the experiment
last_subsubfolder = {'RP_humanoid_bench_lbrsimple','RP_humanoid_bench_lbrsimple_more_constrained'};
% provisory
allpath=which('FindData.m');
global_path=fileparts(allpath);



%% load and coupling data

for iterator_i = 1:length(last_subfolder)
    %% Initializing Variables
    metric1 = cell(1,length(last_subsubfolder));
    metric2 = cell(1,length(last_subsubfolder));
    metric3 = cell(1,length(last_subsubfolder));
    metric4 = cell(1,length(last_subsubfolder));
    metric5 = cell(1,length(last_subsubfolder)); 
    metric7 = cell(1,length(last_subsubfolder));
    for iterator_j = 1:length(last_subsubfolder)
        cur_path =  strcat(global_path,'/',folder,'/',last_subfolder{iterator_i},'/',last_subsubfolder{iterator_j},'/','mn','.mat');
        load(cur_path);
        if(~robotic_flag_per_experiment(iterator_j))
            metric1{iterator_j} = m1;
        end
        metric2{iterator_j} = m2;
        metric3{iterator_j} = m3;
        metric4{iterator_j} = m4;
        metric5{iterator_j} = m5;
        metric7{iterator_j} = m7;
        clearvars m1 m2 m3 m4 m5 m6 m7 robotic_flag;
    end
    %% save new .mat file 
    name_file = [];
    for k = 1:length(last_subsubfolder)
        name_file =  strcat(name_file,last_subsubfolder{k},'_');
    end
    cur_path =  strcat(global_path,'/',folder,'/',last_subfolder{iterator_i},'/',name_file,'.mat');
    save(cur_path,'metric1','metric2','metric3','metric4','metric5','metric7','robotic_flag_per_experiment');
    cur_path = [];
   
end
