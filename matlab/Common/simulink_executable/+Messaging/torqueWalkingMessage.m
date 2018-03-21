classdef torqueWalkingMessage < Messaging.AbstractMessage
    
   methods
       %% important!! params has always to be saved for make the code it works 
       function Pack(obj,controller,params)
           weightRotTask    = controller.alpha{1,1}.sample;
           weightStanceFoot = controller.alpha{1,2}.sample;
           weightSwingFoot  = controller.alpha{1,3}.sample;
           weightPostural   = controller.alpha{1,4}.sample;
           weight_tau       = controller.alpha{1,5}.sample;
   
          % save all the relevant data for the thread
          %% inputData.mat has to be saved in common/simulink_executable      
            save(obj.input2simulink,'weightRotTask','weightStanceFoot','weightSwingFoot', 'weightPostural', 'weight_tau', 'params');
      end
      
      function [s, sd, t] = Unpack(obj,controller,params)
          %% this line has to be present in every unpack function  
          load(obj.outputfromsimulink)
          %% save data 
%           controller.simulation_iterator            = 1;
          controller.simulation_results.task_errors = task_errors.Data;
          controller.simulation_results.joint_error = joint_error.Data;
          controller.simulation_results.tau         = tau_norm.Data;
          controller.simulation_results.exitFlagQP  = exitFlagQP.Data;
          controller.simulation_results.zmp         = ZMP.Data;
          controller.simulation_results.pose_CoM    = pose_CoM.Data;
          controller.simulation_results.pose_lFoot  = pose_lFoot.Data;
          controller.simulation_results.pose_rFoot  = pose_rFoot.Data;
          controller.simulation_results.time        = time;
          s  = s_sim.Data;
          sd = sd_sim.Data;
          t  = time; %params.tStart:params.sim_step:params.tEnd; 
          
      end
      
      function StoreFromSimulink(obj,results)
 
        task_errors          = results.get('task_errors');
        joint_error          = results.get('joint_error');
        tau_norm             = results.get('tau_norm');
        exitFlagQP           = results.get('exitFlagQP');
        ZMP                  = results.get('ZMP');
        pose_CoM             = results.get('pose_CoM');
        pose_lFoot           = results.get('pose_lFoot');
        pose_rFoot           = results.get('pose_rFoot');
        s_sim                = results.get('s');
        sd_sim               = results.get('sd');
        time                 = results.get('tout');
        
        %% simulationResults.mat has to be saved in common/simulink_executable
        save(obj.outputfromsimulink, 'task_errors', 'joint_error', 'tau_norm', 'exitFlagQP', ...
             'pose_CoM', 'ZMP', 'pose_rFoot', 'pose_lFoot', 's_sim', 'sd_sim', 'time');
      end
   end
    
end