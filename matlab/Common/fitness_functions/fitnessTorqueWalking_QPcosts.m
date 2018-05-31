function [fit,failure]  = fitnessTorqueWalking_QPcosts(obj,output)
% fitness function of the icub controlled with the torqueWalking controller
% we try to optimize:
% (minimize)  QP costs over the simulation (the QP cost includes task errors and torque minimization)
% (penalize)  unfeasible optimization with the QP (exit flag = -2, in this case the simulation will end before the final simulation time is reached)
% (penalize)  robot falling/ZMP out of support polygon (in this case, the simulation will end before the final simulation time is reached)
% (penalize)  QP could not be solved at any point during the current rollout (exit flag = {1, -1, -3})
% (constrain) joint limits
% (constrain) torque limits


    %params          = obj.input_4_run{2};
    controller      = obj.input_4_run{4};

    fall_penalty    = -300;  %in this case, I set a very negative penalty because in the unconstrained case i have no lower bound
    exitFlag_penalty= -300;
    
    %task_errors     = controller.simulation_results.task_errors;     %[nsamples x 12] matrix, [CoMx, CoMy, CoMz, OriRot, lFootx,lFooty,lFootz,lFootRot,rFootx, rFooty, rFootz, rFootRot]
    %joint_error     = controller.simulation_results.joint_error;     %[nsamples x nDOF]
    %zmp             = controller.simulation_results.zmp;             %[nsamples x 3]
    %support_polygon = controller.simulation_results.support_polygon; %[2 x 2 x nsamples]
    %feet_in_contact = controller.simulation_results.feet_in_contact; %[nsamples x 2]
    %pose_CoM        = controller.simulation_results.pose_CoM;        %[nsamples x 3]
    %pose_lFoot      = controller.simulation_results.pose_lFoot;      %[nsamples x 3]
    %pose_rFoot      = controller.simulation_results.pose_rFoot;      %[nsamples x 3]
    QP_exitFlag     = controller.simulation_results.QP_exitFlag;     %[nsamples x 1]
    torques         = controller.simulation_results.torques;         %[nsamples x nDOF]
    QP_optObjFunVal = controller.simulation_results.QP_optObjFunVal; %[nsamples x 1]
    time            = controller.simulation_results.time;            %[nsamples x 1]

    t_all           = output{1};
    q_all           = output{2};
    %qd_all          = output{3};
    
    downsample      = 1;
    evaluate_constraints_index = 1;
    
    
    for i=1:downsample:length(time)
        
        res.torques    = torques(i,:);
        q              = q_all(i,:);
        
        %constraint computation
        input_vector = {q(1),q(1),q(2),q(2),q(3),q(3),q(4),q(4),q(5),q(5),q(6),q(6),q(7),q(7),q(8),q(8),q(9),q(9),q(10),q(10),q(11),q(11),q(12),q(12),...
            q(13),q(13),q(14),q(14),q(15),q(15),q(16),q(16),q(17),q(17),q(18),q(18),q(19),q(19),q(20),q(20),q(21),q(21),q(22),q(22),q(23),q(23),...
            res.torques(1),res.torques(1),res.torques(2),res.torques(2),res.torques(3),res.torques(3),res.torques(4),res.torques(4),res.torques(5),res.torques(5),res.torques(6),res.torques(6),res.torques(7),res.torques(7),...
            res.torques(8),res.torques(8),res.torques(9),res.torques(9),res.torques(10),res.torques(10),res.torques(11),res.torques(11),res.torques(12),res.torques(12),...
            res.torques(13),res.torques(13),res.torques(14),res.torques(14),res.torques(15),res.torques(15),res.torques(16),res.torques(16),res.torques(17),res.torques(17),res.torques(18),res.torques(18),res.torques(19),res.torques(19),...
            res.torques(20),res.torques(20),res.torques(21),res.torques(21),res.torques(22),res.torques(22),res.torques(23),res.torques(23)}; %res.exitFlagQP, balance
        
        % here I update the value inside the penalty object
        obj.penalty_handling.EvaluateConstraints(input_vector,evaluate_constraints_index);
        evaluate_constraints_index = evaluate_constraints_index + 1;
    end
 
    %% fitness function computation
    
    %Note: the optimization procedure searches to maximize the fitness,
    %here we want to minimize the QP costs (QP_optObjFunVal)
    fit = - 1/length(QP_optObjFunVal) * (QP_optObjFunVal' * QP_optObjFunVal);
    
    failure = false;
    
    % check to see if the robot has fallen/was about to fall or the QP was unfeasible during the current rollout
    if length(time) < length(t_all)
        disp('for your info: robot has fallen');
        fit = fit - fall_penalty * fall_penalty;
        %failure = true;
    end
    % check to see if the QP could not be solved at any point during the current rollout
    if QP_exitFlag' * QP_exitFlag > 0
        disp('for your info: QP was not always successful');
        fit = fit - exitFlag_penalty * exitFlag_penalty;
    end
    
    %%DEBUG
    fprintf('candidate fit is %f\n', fit)
    %---
    
end
    
