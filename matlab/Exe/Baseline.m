%% baseline execution for compare our method with a random 
%% generation of feasible solutions


clear variables
close all
clc

%% parameters
total_solutions = 200;  % number of solutions that we need 
i = 1;
total_trial = 0;
succ_perfomances = zeros(total_solutions,1);
succ_action      = cell(total_solutions,1);

%% initialize all the data
[bot1,name_scenario,time_struct,time_sym_struct,reference,alphas,controller,constr,inst,~,~,~,cmaes_value_range,qi,qdi,fixed_step,torque_saturation,rawTextFromStorage,name_dat]=Init();



while (i<total_solutions)
    %% Simulation
    %update controllers
    cur_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,controller.GetTotalParamNum()) + cmaes_value_range(1,1)*ones(1,controller.GetTotalParamNum());
    controller.UpdateParameters(cur_action);
    % execute controllers
    tic
    [t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step,'TorqueSat',torque_saturation);
    toc
    %% Evaluate fitness 
    performance = feval(inst.fitness,inst,t,q);
    %compute penalty
    obj.penalty_handling.ComputeConstraintsViolation(-1)
    if(cur_candidates_index == -1)
        performance = performance - inst.penalty_handling.fitness_penalties(1);
    end
    % clean value for the next iteration
    controller.CleanTau();
    controller.CleanTime();
    % if the penalty value is 0 it means that i do not constraints
    % violations
    if(inst.penalty_handling.fitness_penalties(1)==0)
        % save succefull solutions
        succ_perfomances(i) = performance;
        succ_action{i}      = cur_action;
        i = i + 1;
    end
    total_trial = total_trial + 1;
end