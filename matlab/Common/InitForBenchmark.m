function [epsilon,search_space_dimension,explorationRate,cmaes_value_range,...
         n_constraints,constraints_functions,constraints_type,constraints_values,run_function,fitness,clean_function,input]=InitForBenchmark(name_of_file,optim)
    %% parameters
%     allpath = which('FindData.m');
%     path =fileparts(allpath);
%     path = strcat(path,'/configuration_files/',name_of_file);
%     init_func = str2func(path);
%     init_func
      data_storage = str2func(name_of_file);
      data_storage()
    %%  Primary Reference
    % if type_of_task = sampled i have to specify the Time to reach the
    % end of the trajectories that is equal to the simulation time
    reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
    reference.BuildTrajs();
    reference.cur_param_set = numeric_reference_parameter; % some kind of trick to introduce parameters in the optimization procedure
    %% secondary reference
    secondary_refs = References(target_link,traj_type_sec,control_type_sec,geometric_path_sec,geom_parameters_sec,time_law_sec,time_struct,dim_of_task_sec,type_of_traj_sec);
    secondary_refs.BuildTrajs();
    secondary_refs.cur_param_set = numeric_reference_parameter; % some kind of trick to introduce parameters in the optimization procedure
    %% plot scenario
    text = LoadScenario(name_scenario);
    eval(text);

    %% Controller support object
    switch CONTROLLERTYPE
        case 'UF'
            % repellers
            repellers = ContrPart.Repellers(chain_dof,rep_target_link,rep_type,rep_mask,rep_type_of_J_rep,rep_obstacle_ref,single_alpha,J_damp,type_of_rep_strct);
        case 'GHC'
            % constraints
            constraints = ContrPart.Constraints(robots,target_link,constraints_list,constraints_data);
        otherwise
            warning('Unexpected control method')
    end
    %% alpha function
    switch CONTROLLERTYPE
        case 'UF'
            switch choose_alpha
                case 'RBF'
                    % TODO generalize to multichain and generalize respect of controller
                    if(strcmp(combine_rule,'sum'))
                        number_of_action = chains.GetNumTasks(1) ;
                    elseif(strcmp(combine_rule,'projector'))
                        number_of_action = chains.GetNumTasks(1) + repellers.GetNumberOfWeightFuncRep(1);
                    end
                    %---
                    alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,optim);
                case 'constant'
                    alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,value_range_for_optimization_routine,time_struct);
                case 'handTuned'
                    alphas = Alpha.HandTuneAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),starting_value,ti,transition_interval,time_struct);
                otherwise
                    warning('Uexpected alpha functions')
            end
        case 'GHC'
            switch choose_alpha
                case 'chained'
                    alphas = Alpha.ChainedAlpha.BuildCellArray(chains.GetNumChains(),matrix_value,ti,transition_interval,time_struct);
                case 'RBF'
                    % TODO generalize to multichain
                    number_of_action = chains.GetNumTasks(1)*chains.GetNumTasks(1);
                    alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,false);
                    %%---
                otherwise
                    warning('Uexpected alpha functions')
            end
        otherwise
            warning('Unexpected control method')
    end


    %% Controller

    switch CONTROLLERTYPE
        case 'UF'
            controller = Controllers.UF(chains,reference,secondary_refs,alphas,repellers,metric,Param,Param_secondary,combine_rule,regularizer);
        case 'GHC'
            delta_t = time_sym_struct.tf*time_struct.step;
            controller = Controllers.GHC(chains,reference,alphas,constraints,Kp,Kd,regularization,epsilon,delta_t);
        otherwise
            warning('Unexpected control method')
    end
    input{5} = controller;
    search_space_dimension = controller.GetTotalParamNum();
    n_constraints = length(constraints_functions);
end
