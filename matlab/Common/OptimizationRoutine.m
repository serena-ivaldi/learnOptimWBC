% number_of_iteration is usefull only for PlotGraphPaper.m main
function [tau, mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded, name_dat]=OptimizationRoutine(number_of_iteration,n_of_experiment,iter,init_parameters_from_out,name_file)  
    %% initialize all the data
    optim = true;
    [bot1,name_scenario,time_struct,time_sym_struct,simulator_type,reference,alphas,controller,constr,learn_approach,inst,generation_of_starting_point,niter,...
    explorationRate,cmaes_value_range,input,rawTextFromStorage,name_dat]=Init(name_file,optim);
     %% optimization 
     % im using init_value from outside
     switch generation_of_starting_point
         case 'test'
           start_action = user_defined_start_action;
         case 'given'
            start_action = init_parameters_from_out*ones(1,controller.GetTotalParamNum());
         case 'random'
           if(iscell(cmaes_value_range))
               start_action = (cmaes_value_range{2}-cmaes_value_range{1}).*rand(1,controller.GetTotalParamNum()) + cmaes_value_range{1};
           elseif(isvector(cmaes_value_range))
               start_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,controller.GetTotalParamNum()) + cmaes_value_range(1,1)*ones(1,controller.GetTotalParamNum());
           end
         otherwise
               disp('wrong generation_of_starting_point choosed!')
     end
     
     %% do all the checks on the input
     if(controller.GetTotalParamNum() == 0)
        error('the activation policy used do not contains any parameters. Change the activation policy in AllRuntimeParameters to proceed')
     end
     
     %% execution of the optimization routine
     tic
        [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded] = inst.CMAES(controller.GetTotalParamNum(),start_action,niter,explorationRate,cmaes_value_range);
     exec_time = toc
     % analisys of the optimization result for building repertoire
     %[index, search_params ] = flann_build_index(BestActionPerEachGen.policy, struct('algorithm','kmeans','branching',32,'iterations',3,'checks',120)); 
     
     %% collecting results
     BestActionPerEachGen.start_point = start_action;
     %scriptname = 'AllRuntimeParameters';
     % i have to change this number everytime i perform the same test with
     % different runtime parameters
     experiment_number = strcat(num2str(iter),'_of_',num2str(n_of_experiment));
     name_folder = strcat(experiment_number,'_',name_dat);
     % create folder 
     allpath=which('FindData.m');
     path=fileparts(allpath);
     complete_path = strcat(path,'/results/current_experiments/',name_folder);
     mkdir(complete_path)
     % copy runtime parameters in the newly created folder
     fileID = fopen(strcat(complete_path,'/','optimization_parameters.txt'),'w');
     fprintf(fileID,'%s',rawTextFromStorage);
     fclose(fileID);
     % generate graph and data from the current best solution
     %[t_, q, qd]=PlotCmaesResult(complete_path,time_sym_struct,controller,qi,qdi,fixed_step,torque_saturation,name_scenario,time_struct,bestAction,bot1,learn_approach);
     complete_path_to_file = strcat(complete_path,'/data.mat');
     save(complete_path_to_file) 
     % copy name_dat to the base workspace
     assignin('base', 'new_name_folder', strcat(num2str(n_of_experiment),'_',name_dat));
     
     tau = controller.torques;
    

end
