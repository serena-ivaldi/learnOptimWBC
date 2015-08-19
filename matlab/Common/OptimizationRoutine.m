% number_of_iteration is usefull only for PlotGraphPaper.m main
function [tau, mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded, name_dat]=OptimizationRoutine(number_of_iteration,n_of_experiment,iter,init_parameters_from_out,generation_of_starting_point)
  
    
     AllRuntimeParameters
   
    
     %% Reference
     % if type_of_task = sampled i have to specify the Time to reach the
     % end of the trajectories that is equal to the simulation time
     reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
     reference.BuildTrajs();


     %% Load Obstacles 
     text = LoadScenario(name_scenario);
     eval(text);

     close all;

     %% Controller support object
      switch CONTROLLERTYPE
         case 'UF'
            % repellers
            repellers = ContrPart.Repellers(chain_dof,rep_target_link,rep_type,rep_mask,rep_type_of_J_rep,rep_obstacle_ref,single_alpha,J_damp,type_of_rep_strct);
         case 'GHC'
            % constraints
            constraints = ContrPart.Constraints(robots,target_link,constraints_list,constraints_data);
         otherwise
            warning('Unexpected control method');
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
                 alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,true);       
              case 'constant'
                 alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,value_range_for_optimization_routine,time_struct);
              otherwise
                 warning('Uexpected alpha functions')
           end
         case 'GHC'
            switch choose_alpha
                case 'chained'  
                     alphas = Alpha.ChainedAlpha.BuildCellArray(chains.GetNumChains(),matrix_value,ti,transition_interval,time_struct);
                case 'RBF'
                     % TODO generalize to multichain and generalize respect of controller
                     number_of_action = chains.GetNumTasks(1)*chains.GetNumTasks(1);
                     %---
                     alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,true);
               otherwise
                     warning('Uexpected alpha functions')
            end
         otherwise
            warning('Unexpected control method');
     end

     %% Controller 
     switch CONTROLLERTYPE
         case 'UF'
            controller = Controllers.UF(chains,reference,alphas,repellers,metric,Kp,Kd,combine_rule,regularizer,max_time);
        case 'GHC'
            delta_t = time_sym_struct.tf*time_struct.step;
            controller = Controllers.GHC(chains,reference,alphas,constraints,Kp,Kd,regularization,epsilon,delta_t,max_time);
        otherwise
            warning('Unexpected control method');
     end

     %% Instance

     % im using init_value from outside
     switch generation_of_starting_point
        case 'test'
           start_action = [9.274626 9.772934  6.193645  8.097974  12.000000 1.882649  2.484198  9.531073 10.31563 3.863503 8.019803 0.052260 1.705022  1.302329 0.953256];
        case 'given'
            start_action = init_parameters_from_out*ones(1,controller.GetTotalParamNum());
        case 'random'
           start_action = (value_range(1,2)-value_range(1,1)).*rand(1,controller.GetTotalParamNum()) + value_range(1,1)*ones(1,controller.GetTotalParamNum());
        
     end

     inst = Instance(controller,simulator_type,qi,qdi,time_sym_struct,fixed_step,fitness,options);

     tic
     [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded] = inst.CMAES(start_action,niter,explorationRate);
     exec_time = toc
     
     % analisys of the optimization result for building repertoire
     %[index, search_params ] = flann_build_index(BestActionPerEachGen.policy, struct('algorithm','kmeans','branching',32,'iterations',3,'checks',120)); 
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
     PlotCmaesResult(complete_path,time_sym_struct,controller,qi,qdi,fixed_step,torque_saturation,name_scenario,time_struct,bestAction,bot1);
     complete_path_to_file = strcat(complete_path,'/data.mat');
     save(complete_path_to_file) 
     % copy name_dat to the base workspace
     assignin('base', 'new_name_folder', strcat(num2str(n_of_experiment),'_',name_dat));
     
     tau = controller.torques;
    

end
