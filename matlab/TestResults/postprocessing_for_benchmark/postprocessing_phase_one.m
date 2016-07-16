function postprocessing_phase_one()
    clear all 
    close all
    clc

    %% Global Variables
    number_of_test = 40;
    threshold = 2.5;
    %%  Load Path
    % parameter
    folder = 'benckmark';
    % name fo the method
    subfolder = {'CMAES-adaptive'};
    % name of the experiment
    subsubfolder = {'g06','g07','g09','f240','f241','HB'};
    % provisory
    allpath=which('FindData.m');
    global_path=fileparts(allpath);

   

    %% load and coupling data

    for i = 1:length(subfolder)
        for j = 1:length(subsubfolder)
            %% Initializing Variables
            all_m1 = [];
            all_m2 = [];
            all_m2_1 = [];
            all_m3 = [];
            all_m4 = [];
            all_m5 = cell(number_of_test,1);
            all_m6 = [];
            all_m7 = [];
            for k = 1:number_of_test
                cur_path =  strcat(global_path,'/',folder,'/',subfolder{i},'/',subsubfolder{j},'/',num2str(k),'.mat');
                load(cur_path);
                if(~robotic_flag)
                    all_m1 = [all_m1;m1];
                end
                all_m2 = [all_m2;m2];
                all_m2_1 = [all_m2_1;m2_1];
                all_m4 = [all_m4;m4];
                all_m5{k} = m5;
                % compute m3 
                m3 = IndetifySteadyState(m5,threshold);
                all_m3 = [all_m3;m3];
                all_m6 = [all_m6;m6];
                all_m7 = [all_m7;m7];
                cur_path = [];
                clearvars m1 m2 m3 m4 m5 m6 m7 robotic_flag;
            end
            %% save new .mat file 
            cur_path =  strcat(global_path,'/',folder,'/',subfolder{i},'/',subsubfolder{j},'/mn.mat');
            % rename variables
            m1 = all_m1;
            m2 = all_m2;
            m2_1 = all_m2_1;
            m3 = all_m3;
            m4 = all_m4;
            m5 = all_m5;
            m6 = all_m6;
            m7 = all_m7;
            
            fitness_matrix = m5_to_matrix(m5);
            clearvars all_m1 all_m2 all_m2_1 all_m3 all_m4 all_m5 all_m6 all_m7 all_m2_1 ;
            save(cur_path)
            clearvars m1 m2 m3 m4 m5 m6 m7 fitness_matrix;
            cur_path = [];
        end
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


% convert a cell with rows of different length in a matrix  by completing
% the short row with
function out = convert2mat(in)
    maxLength=max(cellfun(@(x)numel(x),in));
    out=cell2mat(cellfun(@(x)cat(2,x,zeros(1,maxLength-length(x))),in,'UniformOutput',false));
    for row = 1:size(out,1)
        for col = 2:size(out,2)        
            if out(row,col) == 0
                out(row,col) = out(row,col-1);
            end
        end
    end
end


function matrix = m5_to_matrix(m5)
    matrix = [];
    for i=1:length(m5)
      matrix = [matrix; m5{i}'];
    end
end


