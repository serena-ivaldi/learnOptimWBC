function postprocessing_phase_zero()
    clear variables 
    close all
    clc

    %% Global Variables
    
    threshold = 2.5;
    starting_experiment = 1;
    ending_experiment = 20;
    number_of_test = ending_experiment - starting_experiment + 1;
    
    %%  Load Path
    % parameter
    folder = 'benckmark';
    % name fo the method
    subfolder = {'CMAES-adaptive'};
    % name of the experiment
    subsubfolder = {'RP_humanoid_bench_lbrsimple_more_constrained'};
    % provisory
    allpath=which('FindData.m');
    global_path=fileparts(allpath);
    optim = false;
    
    
    %% Initializing Variables
    all_m1 = [];
    all_m2 = [];
    all_m2_1 = [];
    all_m3 = [];
    all_m4 = [];
    all_m5 = cell(number_of_test,1);
    all_m6 = [];
    all_m7 = [];

    %% load and complete data
    for k = starting_experiment:ending_experiment
        fprintf('EXPERIMENT %d\n',k)
        cur_path =  strcat(global_path,'/',folder,'/',subfolder{1},'/',subsubfolder{1},'/',num2str(k),'.mat');
        load(cur_path);
        
        % because of a bug I had to repeat the best to compute the other
        % metrics
        [~,~,~,~,~,~,~,~,~,~,inst,~,~,~,~,~,~,~,~]=Init(subsubfolder{1},optim);
        % at the beggining we have to perform a check to see if the
        % number of -1 one is more than half the element of fitness
        % in this case we lose the steady state value and the perfomance so
        % we need to signal wich of the value lack of data
        ind_to_check_feasibility = find(m5 == -1);
        not_compute_m3_and_signal_sample = false;
        if(length(ind_to_check_feasibility)>(length(m5)/2))
            not_compute_m3_and_signal_sample = true;
            fprintf('the data %d.mat is invalid for an excess of error in the mean perfomances\n',k)
        end
        % we have to repeat this check for vanilla and adaptive only
        % because i changed the penalty for them
        ind_to_check_feasibility = [];
        ind_to_check_feasibility = find(m5 == -100000);
        not_compute_m3_and_signal_sample = false;
        if(length(ind_to_check_feasibility)>(length(m5)/2))
            not_compute_m3_and_signal_sample = true;
            fprintf('the data %d.mat is invalid for an excess of error in the mean perfomances\n',k)
        end
        
        %% for corrected this part should be useless
        % only for cmaes-vanilla and cmaes-adaptive i have to interpolate the
        % fitness to remove -1 (they are integration fail or the take too long)
        if((strcmp(subfolder{1},'(1+1)CMAES-vanilla') || strcmp(subfolder{1},'fmincon-fmincon'))  && ~not_compute_m3_and_signal_sample)
            % check if the first or the last ar -1 in that case i just
            % copy the second value for the first and the one before
            % last for the last value
            if(m5(1) == -1)
                m5(1) = m5(2);
            end
            if(m5(end) == -1)
                m5(end) = m5(end-1);
            end
            ind = find(m5 == -1);
            if (~isempty(ind))
                % remove -1 from the sequence
                app_vector = m5;
                app_vector(ind) = [];
                shift = 0:length(ind) - 1;
                query_point = (ind-shift) - 0.5;
                v = 1:length(app_vector);
                vq = interp1(v,app_vector,query_point);
                m5(ind) = vq;
            end
        end
        
        %% interpolation of the fitness for an integration error  
        % only for cmaes-vanilla and cmaes-adaptive i have to interpolate the
        % fitness to remove -1 (they are integration fail or the take too long)
        if((strcmp(subfolder{1},'CMAES-vanilla') || strcmp(subfolder{1},'CMAES-adaptive'))  && ~not_compute_m3_and_signal_sample)
            % check if the first or the last ar -1 in that case i just
            % copy the second value for the first and the one before
            % last for the last value
            if(m5(1) == -100000)
                m5(1) = m5(2);
            end
            if(m5(end) == -100000)
                m5(end) = m5(end-1);
            end
            ind = find(m5 == -100000);
            if (~isempty(ind))
                % remove -1 from the sequence
                app_vector = m5;
                app_vector(ind) = [];
                shift = 0:length(ind) - 1;
                query_point = (ind-shift) - 0.5;
                v = 1:length(app_vector);
                vq = interp1(v,app_vector,query_point);
                m5(ind) = vq;
            end
        end

        %% if the data is fine i can compute m3
        if(~not_compute_m3_and_signal_sample)
            % compute m3 
            m3 = IndetifySteadyState(m5,threshold);
        end
         %% recording all the violation for the best action 
        [c, ceq, perfomance] = inst.computeConstr(m6);
        % all the violation for the best action 
        m2_1 =  inst.penalty_handling.penalties(1,:);
        if(~strcmp(subfolder{1},'fmincon-fmincon'))
            % sum of  all the violations
            m2 = sum(abs((c > 0).*c)) + sum(abs((ceq ~= 0).*ceq));
        end
        %% sometimes m7 could be -1 due to an integration error due  to the
        % max time computation and due to the fact that there was an error
        % in the fitness function
        if(m7 == -1  || m7 == -100000 || strcmp(subfolder{1},'CMAES-vanilla'))
            m7 = perfomance;
        end
               
         %% building the complete dataset for the current method and experiment
        
        if(~robotic_flag)
            all_m1 = [all_m1;m1];
        end
        all_m2 = [all_m2;m2];
        all_m2_1 = [all_m2_1;m2_1];
        all_m4 = [all_m4;m4];
        if(isrow(m5))
            m5 =  m5';
        end
        all_m5{k} = m5;
        % compute m3 
        m3 = IndetifySteadyState(m5,threshold);
        all_m3 = [all_m3;m3];
        all_m6 = [all_m6;m6];
        all_m7 = [all_m7;m7];
       
        
        cur_path = []; 
        clearvars m1 m2 m2_1 m3 m4 m5 m6 m7 robotic_flag c ceq perfomance not_compute_m3_and_signal_sample n_constraints fitness_correction_for_cmaes_vanilla
        close all;
    end
    
    %% save new .mat file 
    cur_path =  strcat(global_path,'/',folder,'/',subfolder{1},'/',subsubfolder{1},'/mn.mat');
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
    
    clearvars all_m1 all_m2 all_m2_1 all_m3 all_m4 all_m5 all_m6 all_m7 all_m2_1;
    % to display in a nicer way the evolution of the fitness for all the
    % results
    
    save(cur_path)
    
    
        
    
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


