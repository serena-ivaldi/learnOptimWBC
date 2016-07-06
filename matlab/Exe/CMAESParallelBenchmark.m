
function CMAESParallelBenchmark
    %% test main for the cmaes optimizer
    %% this file is conceived for testing one method with different test functions
    clear variables
    close all
    clc

    matlabpool
    
    %% DATA 1
    robotics_experiment = [1]; % series of value that say if the current experiment is a robotics experiments or not
    niter_tot = 400;  %number of functions evaluations
    function_2_test ={'RP_humanoid_bench_lbrsimple'};%'robotic_experiments','g06','g07','g09','f240','f241','HB'}; % 
    learn_approach = 'CMAES'; %CMAES (1+1)CMAES  CEM,fmincon    with (1+1)CMAES i have to use vanilla constraints management  (temporary)
    method_to_use = 'vanilla';  % adaptive , vanilla ,empty,fmincon

    repetition_of_the_experiment = 20; % at least 2
    threshold = 2.5; % value to identify the beginning of steady state
    % the threshold is express in %, means +/- 2,5% from the steady value

    % starting value of parameters
    generation_of_starting_point = 'test'; % 'test', 'given', 'random'
    


     %%  SAVE PATH
    allpath=which('FindData.m');
    local_path=fileparts(allpath);
    local_path = strcat(local_path,'/benckmark');
    name_folder = strcat(learn_approach,'-',method_to_use);
    mkdir(local_path,name_folder);
    local_path = strcat(local_path,'/',name_folder);


    %% INSTANCE PARAMETER
    number_of_function_2_test = length(function_2_test);
    for jj=1:number_of_function_2_test
        % here i create the local folder to store all the intermediate
        % results
        name_local_folder = function_2_test{jj};
        mkdir(local_path,name_local_folder);
        sub_local_path = strcat(local_path,'/',name_local_folder);
        % i set the value to pass to all the workers
        cur_function = function_2_test{jj};
        cur_robotic_experiment = robotics_experiment(jj);
    %% PARALLEL FOR
        parfor iter=1:repetition_of_the_experiment
            ParallelBechmarkRoutine(sub_local_path,learn_approach,method_to_use,cur_function,iter,cur_robotic_experiment,generation_of_starting_point,niter_tot,threshold)  
        end
    end
    
    matlabpool close
end
    
    












% function steady_state_begin = IndetifySteadyState(vector,tresh)
%     steady_value = vector(end);
%     keep_search = true;
%     for zzz = 1:length(vector)
%         if(abs(steady_value-vector(zzz))<(tresh/100*steady_value) && keep_search)
%             steady_state_begin = zzz;
%             keep_search = false;
%         end
%     end
% end
% 
% 
% % convert a cell with rows of different length in a matrix  by completing
% % the short row with
% function out = convert2mat(in)
%     maxLength=max(cellfun(@(x)numel(x),in));
%     out=cell2mat(cellfun(@(x)cat(2,x,zeros(1,maxLength-length(x))),in,'UniformOutput',false));
%     for row = 1:size(out,1)
%         for col = 2:size(out,2)        
%             if out(row,col) == 0
%                 out(row,col) = out(row,col-1);
%             end
%         end
%     end
% end
