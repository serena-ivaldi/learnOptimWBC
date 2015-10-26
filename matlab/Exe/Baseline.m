%% baseline execution for compare our method with a random 
%% generation of feasible solutions

clear variables
close all
clc

%% parameters
total_solutions = 4;  % number of solutions that we need 
i = 1;
total_trial = 0;
succ_perfomances = zeros(total_solutions,1);
succ_action      = cell(total_solutions,1);

%% initialize all the data
[bot1,name_scenario,time_struct,time_sym_struct,reference,alphas,controller,constr,inst,~,~,~,cmaes_value_range,qi,qdi,fixed_step,torque_saturation,rawTextFromStorage,name_dat]=Init();

matlabpool Local 

spmd
    C_succ_perfomances = codistributed(succ_perfomances,codistributor1d(1));
    C_succ_action = codistributed(succ_action,codistributor1d(1));
    L_succ_perfomances = getLocalPart(C_succ_perfomances);
    L_succ_action = getLocalPart(C_succ_action);
    
    
    while (i<length(L_succ_perfomances) + 1)
        %% Simulation
        %update controllers
        cur_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,controller.GetTotalParamNum()) + cmaes_value_range(1,1)*ones(1,controller.GetTotalParamNum());
        controller.UpdateParameters(cur_action);
        % execute controllers

        try
            tic
            [t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step,'TorqueSat',torque_saturation);
            toc
            %% Evaluate fitness 
            performance = feval(inst.fitness,inst,t,q);
            %compute penalty
            inst.penalty_handling.ComputeConstraintsViolation(-1)
            performance = performance - inst.penalty_handling.fitness_penalties(1);

            % clean value for the next iteration
            controller.CleanTau();
            controller.CleanTime();
            % if the penalty value is 0 it means that i do not constraints
            % violations
            if(inst.penalty_handling.fitness_penalties(1)==0)
                % save succefull solutions
                L_succ_perfomances(i)  = performance;
                L_succ_action{i}       = cur_action;
                i = i + 1;
            end
        catch err
            disp('integration failed')
            controller.CleanTau(); 
            controller.CleanTime();
        end
        total_trial = total_trial + 1;
    end
end


%% compose result in unique vector
final_total_trial = 0;
for i =1:length(total_trial)
    final_total_trial = final_total_trial + total_trial{i};
end

final_succ_perfomances=[];
for i =1:length(L_succ_perfomances)
    final_succ_perfomances = [final_succ_perfomances ; L_succ_perfomances{i}'];
end

final_succ_action=[];
for i =1:length(L_succ_action)
    final_succ_action = [final_succ_action ; L_succ_action{i}'];
end

