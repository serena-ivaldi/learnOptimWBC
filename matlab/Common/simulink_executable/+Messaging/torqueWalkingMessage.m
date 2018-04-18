classdef torqueWalkingMessage < Messaging.AbstractMessage
    
   methods
       %% important!! params must always be saved to make the code work
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
%           controller.simulation_iterator                = 1;
          controller.simulation_results.task_errors     = task_errors.Data;
          controller.simulation_results.joint_error     = joint_error.Data(:,:)';
          controller.simulation_results.torques         = torques.Data; %this is a timeseries of the vector of joint torques
          controller.simulation_results.exitFlagQP      = exitFlagQP.Data;
          controller.simulation_results.zmp             = ZMP.Data;
          controller.simulation_results.pose_CoM        = pose_CoM.Data(:,:)';
          controller.simulation_results.support_polygon = permute(support_polygon.Data, [3 1 2]); %this is a matrix of the size (nsamples, 2,2)
          controller.simulation_results.feet_in_contact = feet_in_contact.Data(:,:)';
          controller.simulation_results.pose_lFoot      = pose_lFoot.Data(:,:)';
          controller.simulation_results.pose_rFoot      = pose_rFoot.Data(:,:)';
          controller.simulation_results.time            = time;
          s  = s_sim.Data;
          sd = sd_sim.Data;
          t  = params.tStart:params.sim_step:params.tEnd; 
          
      end
      
      function StoreFromSimulink(obj,results)
 
        task_errors          = results.get('task_errors');
        joint_error          = results.get('joint_error');
        torques              = results.get('torques');
        exitFlagQP           = results.get('exitFlagQP');
        ZMP                  = results.get('ZMP');
        feet_in_contact      = results.get('feet_in_contact');
        support_polygon      = results.get('support_polygon');
        pose_CoM             = results.get('pose_CoM');
        pose_lFoot           = results.get('pose_lFoot');
        pose_rFoot           = results.get('pose_rFoot');
        s_sim                = results.get('s');
        sd_sim               = results.get('sd');
        time                 = results.get('tout');
        
        %% simulationResults.mat has to be saved in common/simulink_executable
        save(obj.outputfromsimulink, 'task_errors', 'joint_error', 'torques', 'exitFlagQP', ...
            'ZMP', 'feet_in_contact', 'support_polygon', 'pose_CoM', 'pose_lFoot', 'pose_rFoot', 's_sim', 'sd_sim', 'time');
      end
   end
    
end