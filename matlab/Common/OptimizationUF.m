function [tau,init_parameters,mean_performances, bestAction, policies, costs, succeeded]=OptimizationUF(iter,init_parameters,random)

    AllRuntimeParameters
    %% TODO
    %% randomization init parameters
    %% assigned a specific value as starting point
    
    switch CONTROLLERTYPE
    case 'UF'
        %% Reference
        % if type_of_task = sampled i have to specify the Time to reach the
        % end of the trajectories that is equal to the simulation time
        reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
        reference.BuildTrajs();


        %% Load Obstacles 
        text = LoadScenario(name_scenario);
        eval(text);

        close all;

        %% repellers
        repellers = ContrPart.Repellers(chain_dof,rep_target_link,rep_type,rep_mask,rep_type_of_J_rep,rep_obstacle_ref,single_alpha,type_of_rep_strct); 

        %% alpha function

        % TODO generalize to multichain and generalize respect of controller
        if(strcmp(combine_rule,'sum'))
            number_of_action = chains.GetNumTasks(1);
        elseif(strcmp(combine_rule,'projector'))
            number_of_action = chains.GetNumTasks(1) + repellers.GetNumberOfWeightFuncRep(1);
        end
        %---

        alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,range,precomp_sample,numeric_theta,true);       
        %alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);

        %% Controller
        controller = Controllers.UF(chains,reference,alphas,repellers,metric,Kp,Kd,combine_rule,regularizer,max_time);

        %% Instance
        controller.GetTotalParamNum()
        start_action = init_parameters*ones(1,controller.GetTotalParamNum());

        inst = Instance(controller,simulator_type,qi,qdi,time_sym_struct,fixed_step,fitness,options);
        [mean_performances ,bestAction ,policies ,costs ,succeeded] = inst.CMAES(start_action,niter,explorationRate);

        scriptname = 'RuntimeVariable';
        % i have to change this number everytime i perform the same test with
        % different runtime parameters
        experiment_number = iter;
        name_folder = strcat(experiment_number,'__',name_dat);
        complete_path=PlotCmaesResult(time_struct,controller,bestAction,scriptname,name_folder);
        complete_path_to_file= strcat(complete_path,'/data.mat');
        save(complete_path_to_file) 

        tau = controller.torques;
    case 'GHC'
       % constraints
       disp('iterative optimization not possible for GHC')
    otherwise
        warning('Unexpected control method');
    end

end