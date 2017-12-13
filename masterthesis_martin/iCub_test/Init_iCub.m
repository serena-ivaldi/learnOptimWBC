function [bot1,name_scenario,time_struct,time_sym_struct,reference,alphas,controller,constr,inst,generation_of_starting_point, ...
         niter,explorationRate,cmaes_value_range,qi,qdi,fixed_step,torque_saturation,rawTextFromStorage,name_dat] = Init_iCub()

AllRuntimeParameters

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();
reference.cur_param_set = numeric_reference_parameter;
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
              alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,false);
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
        %controller = Controllers.UF(chains,reference,alphas,repellers,metric,Kp,Kd,combine_rule,regularizer,max_time);

        import WBM.*

        %% Initialization of the WBM:
        %
        % base model parameters:
        iCub_model = wbmBaseModelParams;
        iCub_model.urdfRobot    = 'icubGazeboSim';
        %iCub_model.urdfLinkName = 'l_sole';
        iCub_model.wf_R_rootLnk = eye(3,3);
        iCub_model.wf_p_rootLnk = zeros(3,1);
        iCub_model.g_wf         = [0; 0; -9.81];
        % base robot config:
        iCub_config = wbmHumanoidConfig;
        iCub_config.ndof          = 25;
        iCub_config.nCstrs        = 2;
        iCub_config.cstrLinkNames = {'l_sole', 'r_sole'};
        iCub_config.dampCoeff     = 0.75;
        % body positions of the iCub-Robot (in degrees):
        % (this configuration assumes an iCub-Robot with 25 DoFs.)
        iCub_config.pos_torso    = [-10.0; 0.0; 0.0];
        iCub_config.pos_leftArm  = [-19.7; 29.7; 0.0; 44.9; 0.0];
        iCub_config.pos_leftLeg  = [25.5; 0.1; 0.0; -38.5; -5.5; -0.1];
        iCub_config.pos_rightArm = iCub_config.pos_leftArm;
        iCub_config.pos_rightLeg = iCub_config.pos_leftLeg;
        % init-state parameters:
        iCub_config.initStateParams.x_b     = zeros(3,1);
        iCub_config.initStateParams.qt_b    = zeros(4,1);
        iCub_config.initStateParams.q_j     = [iCub_config.pos_torso; iCub_config.pos_leftArm; iCub_config.pos_rightArm; ...
                                               iCub_config.pos_leftLeg; iCub_config.pos_rightLeg] * (pi/180.0); % in radians
        iCub_config.initStateParams.dx_b    = zeros(3,1);
        iCub_config.initStateParams.omega_b = zeros(3,1);
        iCub_config.initStateParams.dq_j    = zeros(iCub_config.ndof,1);

        % init the mex-WholeBodyModel for the iCub-Robot:
        wf2FixLnk = true;
        wbm_iCub  = WBM(iCub_model, iCub_config, wf2FixLnk);

        controller = Controllers.UF_iCub(chains, reference, alphas, repellers, metric, Kp, Kd, combine_rule, regularizer, max_time, wbm_iCub);
    case 'GHC'
      delta_t = time_sym_struct.tf*time_struct.step;
      controller = Controllers.GHC(chains,reference,alphas,constraints,Kp,Kd,regularization,epsilon,delta_t,max_time);
    otherwise
        warning('Unexpected control method')
end

 %% Constraints
constr=Optimization.FixPenalty(controller.GetTotalParamNum(),constraints_functions,constraints_type,constraints_values);

 %% Instance
input{5} = controller;
inst = Optimization.Instance(constr,run_function,fitness,clean_function,input);

end