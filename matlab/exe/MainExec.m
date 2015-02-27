clear variables
close all
clc

AllRuntimeParameters

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

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
        % TODO generalize to multichain 
        if(strcmp(combine_rule,'sum'))
            number_of_action = chains.GetNumTasks(1);
        elseif(strcmp(combine_rule,'projector'))
            number_of_action = chains.GetNumTasks(1) + repellers.GetNumberOfWeightFuncRep(1);
        end
        %%--- 

        alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,false);       
        %alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);  
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
        controller = Controllers.UF(chains,reference,alphas,repellers,metric,Kp,Kd,combine_rule,regularizer,max_time);
    case 'GHC'
      delta_t = time_sym_struct.tf*time_struct.step;
      controller = Controllers.GHC(chains,reference,alphas,constraints,Kp,Kd,regularization,epsilon,delta_t,max_time);
    otherwise
        warning('Unexpected control method')
end


%% Simulation
tic
[t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step,'TorqueSat',torque_saturation);
toc
%% Display
fps = 200;
video = false;
step_save_fig = 20;
save_fig = true;
ee_trajectory = true; 
elbow_traj    = true;
%---

% plot the trajectory of the elbow or e-e or both
if(ee_trajectory || elbow_traj)
   
   [ee,elbow] = ComputePositions(q{1},t,controller);
   ee = ee';
   elbow = elbow';
   izy = 1;
   handle_vector = [];
   if(ee_trajectory)
      name_of_trace {1,izy} = 'end-effector';  
      handle1 = plot3(ee(:,1)',ee(:,2)',ee(:,3)','Color','r','LineWidth',2);
      handle_vector=[handle_vector,handle1];
      izy = izy + 1;
   end
   if(elbow_traj)
      name_of_trace{1,izy} = 'elbow';  
       handle2 = plot3(elbow(:,1)',elbow(:,2)',elbow(:,3)','Color','g','LineWidth',2);
       handle_vector=[handle_vector,handle2];
      izy = izy + 1;
   end
   hold on;
   hL = legend(handle_vector,name_of_trace);
   set(hL,'FontSize',15);
end


if(~video && ~save_fig)
   zoom =  5.0698;
   set(gca,'CameraViewAngle',zoom);
   camera_position = [14.3762    9.7004   15.0093];
   campos(camera_position)
   bot1.plot(q{1},'fps',fps);
elseif(video)
   %at the end of the video simulation after chosing a good camera pos and
   %zoom
   % to see camera position call "campos" on the shell 
   % to see zoom call "get(gca,'CameraViewAngle')" on the shell
   allpath = which('FindData.m');
   path = fileparts(allpath);
   path = strcat(path,'/video');
   camera_position = [-7.5371   -1.1569   21.1612];
   zoom =  2.4702;
   set(gca,'CameraViewAngle',zoom);
   campos(camera_position)
   bot1.plot(q{1},'movie',path);
elseif(save_fig)
   camera_position = [-7.5371   -1.1569   21.1612];
   zoom =  2.4702;
   set(gca,'CameraViewAngle',zoom);
   campos(camera_position)
   SaveFigures(bot1,q{1},step_save_fig)
end




