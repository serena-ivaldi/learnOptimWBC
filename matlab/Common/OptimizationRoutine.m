% number_of_iteration is usefull only for PlotGraphPaper.m main
function [tau,mean_performances, bestAction, policies, costs, succeeded]=OptimizationRoutine(number_of_iteration,n_of_experiment,iter,init_parameters_from_out,random)
  
    
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
            constraints = ContrPart.Constraints(constraints_list,constraints_data);
         otherwise
            warning('Unexpected control method');
      end


     %% alpha function
     switch CONTROLLERTYPE
         case 'UF'
           % TODO generalize to multichain and generalize respect of controller
           if(strcmp(combine_rule,'sum'))
               number_of_action = chains.GetNumTasks(1);
           elseif(strcmp(combine_rule,'projector'))
               number_of_action = chains.GetNumTasks(1) + repellers.GetNumberOfWeightFuncRep(1);
           end
           %---

           alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,true);       
           %alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);
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
     start_action = init_parameters_from_out*ones(1,controller.GetTotalParamNum());
     if(random)
            
        start_action = (value_range(1,2)-value_range(1,1)).*rand(1,controller.GetTotalParamNum()) + value_range(1,1)*ones(1,controller.GetTotalParamNum());
       
     end

     inst = Instance(controller,simulator_type,qi,qdi,time_sym_struct,fixed_step,fitness,options);

     tic
     [mean_performances ,bestAction ,policies ,costs ,succeeded] = inst.CMAES(start_action,niter,explorationRate);
     exec_time = toc

     %scriptname = 'AllRuntimeParameters';
     % i have to change this number everytime i perform the same test with
     % different runtime parameters
     experiment_number = strcat(num2str(iter),'_of_',num2str(n_of_experiment));
     name_folder = strcat(experiment_number,'_',name_dat);
     complete_path=PlotCmaesResult(time_struct,controller,bestAction,rawTextFromStorage,name_folder);
     complete_path_to_file = strcat(complete_path,'/data.mat');
     save(complete_path_to_file) 

     tau = controller.torques;
    

end
