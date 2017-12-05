%% this method is only suitable for (1+1)CMAES-vanilla (only for the fitness visualization) and fmincon-fmincon and BO-nopenalty
%% in this method we rielaborate the fitness value evolution of (1+1)cmaes and fmincon to 
%% to take into account the difference between function evaluation and generation
%% in this way we can compare cmaes with (1+1)cmaes and fmincon

% RP_humanoid_bench_lbrsimple_more_constrained number_of_experiments = 20  number_of_generations_of_other_methods = 33 number_of_function_evaluation = 400
% RP_humanoid_bench_lbrsimple number_of_experiments = 20  number_of_generations_of_other_methods = 33 number_of_function_evaluation = 400
% f240                 number_of_experiments = 40  number_of_generations_of_other_methods = 167 number_of_function_evaluation = 1500
% f241                 number_of_experiments = 40  number_of_generations_of_other_methods = 167 number_of_function_evaluation = 1500
% g06                  number_of_experiments = 40  number_of_generations_of_other_methods = 250 number_of_function_evaluation = 1500
% g07                  number_of_experiments = 40  number_of_generations_of_other_methods = 136 number_of_function_evaluation = 1500
% g09                  number_of_experiments = 40  number_of_generations_of_other_methods = 150 number_of_function_evaluation = 1500
% HB                   number_of_experiments = 40  number_of_generations_of_other_methods = 167 number_of_function_evaluation = 1500

function postprocessing_phase_one_dot_five()

    clear variables 
    close all
    clc

    %% global variables
    tresh = 2.5;
    number_of_experiments = 45;
    % one for each entry of the newsubsubfolder variable
    number_of_generations_of_other_methods = [167 167 250 136 150 167];
    number_of_function_evaluation = 300;
    %%  Load Path
    % parameter
    newfolder = 'benckmark';
    % name fo the method
    newsubfolder = {'BO-nopenalty'};
    % name of the experiment
    newsubsubfolder = {'g07','g09','HB'}; % 'f240','f241','g06','g07','g09','HB' 'RP_humanoid_bench_lbrsimple','RP_humanoid_bench_lbrsimple_more_constrained'
    % provisory
    allpath=which('FindData.m');
    newglobal_path=fileparts(allpath);


    for ijk = 1:length(newsubfolder)
            for jjj = 1:length(newsubsubfolder)
                cur_path =  strcat(newglobal_path,'/',newfolder,'/',newsubfolder{ijk},'/',newsubsubfolder{jjj},'/','mn.mat');
                load(cur_path);
                coeffiecient = round(number_of_function_evaluation/number_of_generations_of_other_methods(jjj));
                % the two variable that we have to update are m3(steady
                % state solution) and m5(all the fitness)

                
                %% updating m5
                
                % only for fmincon if the number of interation of the
                % fitness vector are shorter because fmincon ends earlier
                % we need to copy the last value of the fitness in order to
                % have something that is comparable with the other methods
                if(strcmp(newsubfolder,'fmincon-fmincon'))
                    %% we need to change the sign of the fitness on ly for fmincon because the other method (cmaes like) find a maximum
                    m5{1} = -m5{1};
                    m7 = -m7;
                    last_fitness = m5{1}(end);
                    sp = length(m5{1});
                    if(length(m5{1})<number_of_function_evaluation)
                        for zz= (sp + 1) : number_of_function_evaluation
                            m5{1}(zz) = last_fitness;
                        end
                    end
                end
                
                
                 %% updating m3
                if(strcmp(newsubfolder,'fmincon-fmincon') || strcmp(newsubfolder,'BO-nopenalty'))
                    % i have to compute this quantty here for fmincon 
                    % because i do not run on it the phase_one
                    % postprocessing
                    m3 = IndetifySteadyState(m5{1},tresh);
                    m3 = round(m3/coeffiecient);
                else
                    m3 = round(m3/coeffiecient);
                end
                
                %% downsampling m5 in order to have a comparable fitness
                app_vector= [];
                app_cell_array=cell(length(m5),1);
                
                % because i want to keep the best value i will downsample
                % the fitness vector starting from the end
                for k = 1:length(m5)
                    iter = 1;
                    for kk = number_of_function_evaluation : -coeffiecient : 1
                        if(iter <= number_of_generations_of_other_methods(jjj))
                            app_vector = [app_vector;m5{k,1}(kk,1)];
                        end
                        iter = iter + 1;
                    end
                    app_vector = flipud(app_vector);
                    app_cell_array{k}=app_vector;
                    app_vector = [];
                end
                %% update m5
                m5 = app_cell_array;
                cur_path=[];
                %% update fitness to matrix for a correct visualization of
                % the result
                fitness_matrix = m5_to_matrix(m5);
                
                %% only for fmincon i need to do the padding of the metrics to be compliant with 
                %% the dimnesion of the other method. i have only one repetition for fmincon-fmincon 
                %% because it is a deterministic method
                if(strcmp(newsubfolder,'fmincon-fmincon') || strcmp(newsubfolder,'BO-nopenalty'))
                    if(~isempty(m1))
                        m1 = m1*ones(number_of_experiments,1);
                    end
                    m2 = m2*ones(number_of_experiments,1);
                    m3 = m3*ones(number_of_experiments,1);
                    m4 = m4*ones(number_of_experiments,1);
                    m7 = m7*ones(number_of_experiments,1);
                end
                

                %% save new .mat file 
                cur_path =  strcat(newglobal_path,'/',newfolder,'/',newsubfolder{ijk},'/',newsubsubfolder{jjj},'/mn_new.mat');
                save(cur_path)

            end

    end
end


function matrix = m5_to_matrix(m5)
    matrix = [];
    for i=1:length(m5)
      matrix = [matrix; m5{i}'];
    end
end

function steady_state_begin = IndetifySteadyState(vector,tresh)
    steady_value = vector(end);
    keep_search = true;
    for zzz = 1:length(vector)
        if(abs(steady_value-vector(zzz))<abs(tresh/100*steady_value) && keep_search)
            steady_state_begin = zzz;
            keep_search = false;
        end
    end
end
