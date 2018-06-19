classdef torqueWalkingRbfMessage < Messaging.AbstractMessage
    
   methods
       %% important!! params must always be saved to make the code work
       function Pack(obj,controller,params)
           
           time = params.tStart:params.sim_step:params.tEnd;
           
           
           weightStanceFoot = zeros(length(time),1);
           weightSwingFoot  = zeros(length(time),1);
           weightHand       = zeros(length(time),1);
           weightRotTask    = zeros(length(time),1);
           weightPostural   = zeros(length(time),1);
           weight_tau       = zeros(length(time),1);
           
           index = 1;
           for ti=time
               weightStanceFoot(index) = controller.alpha{1,1}.GetValue(ti);
               weightSwingFoot(index)  = controller.alpha{1,2}.GetValue(ti);
               weightHand(index)       = controller.alpha{1,3}.GetValue(ti);
               weightRotTask(index)    = controller.alpha{1,4}.GetValue(ti);
               weightPostural(index)   = controller.alpha{1,5}.GetValue(ti);
               weight_tau(index)       = controller.alpha{1,6}.GetValue(ti);
               index = index + 1;
           end
           
          
          % save all the relevant data for the thread
          %% inputData.mat has to be saved in common/simulink_executable      
            save(obj.input2simulink,'weightStanceFoot','weightSwingFoot', 'weightHand', 'weightRotTask', 'weightPostural', 'weight_tau', 'params');
      end
      
      function [s, sd, t] = Unpack(obj,controller,params)
          %% this line has to be present in every unpack function  
          load(obj.outputfromsimulink)
          %% save data 
%         controller.simulation_iterator                = 1;
          controller.simulation_results.task_errors     = task_errors.Data(:,:)';
          controller.simulation_results.joint_error     = joint_error.Data(:,:)';
          controller.simulation_results.torques         = torques.Data; %this is a timeseries of the vector of measured joint torques
          controller.simulation_results.QP_optObjFunVal = QP_optObjFunVal.Data;
          controller.simulation_results.QP_exitFlag     = QP_exitFlag.Data;
          controller.simulation_results.zmp             = ZMP.Data;
          controller.simulation_results.pose_CoM        = pose_CoM.Data(:,:)';
          controller.simulation_results.support_polygon = support_polygon.Data; %this is a matrix of the size (2,2,nsamples)
          controller.simulation_results.feet_in_contact = feet_in_contact.Data(:,:)';
          controller.simulation_results.pose_lFoot      = pose_lFoot.Data(:,:)';
          controller.simulation_results.pose_rFoot      = pose_rFoot.Data(:,:)';
          controller.simulation_results.time            = time;
          controller.simulation_results.zmpErr          = zmpErr.Data;
          controller.log.external_wrench                = external_wrench;
          
          
          
          s  = s_sim.Data;
          sd = sd_sim.Data;
          t  = params.tStart:params.sim_step:params.tEnd; 
          
      end
      
      function StoreFromSimulink(obj,results,params)
 
        task_errors          = results.get('task_errors');
        joint_error          = results.get('joint_error');
        torques              = results.get('torques');
        QP_optObjFunVal      = results.get('QP_optObjfun_value');
        QP_exitFlag          = results.get('QP_exitFlag');
        ZMP                  = results.get('ZMP');
        feet_in_contact      = results.get('feet_in_contact');
        support_polygon      = results.get('support_polygon');
        pose_CoM             = results.get('pose_CoM');
        pose_lFoot           = results.get('pose_lFoot');
        pose_rFoot           = results.get('pose_rFoot');
        s_sim                = results.get('s');
        sd_sim               = results.get('sd');
        time                 = results.get('tout');
        zmpErr               = results.get('zmpErr');
        external_wrench      = params.external_force;
        
        
        %% simulationResults.mat has to be saved in common/simulink_executable
        save(obj.outputfromsimulink, 'task_errors', 'joint_error', 'torques', ...
            'QP_optObjFunVal', 'QP_exitFlag', ...
            'feet_in_contact', 'ZMP', 'support_polygon', ...
            'pose_CoM', 'pose_lFoot', 'pose_rFoot', 's_sim', 'sd_sim', 'time','zmpErr','external_wrench');
      end
   end
    
end