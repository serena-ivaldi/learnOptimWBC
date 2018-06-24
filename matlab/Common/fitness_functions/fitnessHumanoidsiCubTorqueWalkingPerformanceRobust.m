function [fit,failure]  = fitnessHumanoidsiCubTorqueWalkingPerformanceRobust(obj,output)
% fitness function of the icub controlled with the torqueWalking controller
% we try to optimize:
% (minimize) error of desired task values (and posture error if desired)
% (minimize) joint torques 
% (maximize) distance between ZMP and bounds of the support polygon
% (minimize) joint torques
% (penalize) unfeasible optimization with the QP (exit flag = -2, in this case the simulation will end before the final simulation time is reached)
% (penalize) failure, such as the robot falling (in this case, the simulation will end before the final simulation time is reached)
% (constrain) QP could not be solved (exit flag ~= 0)
% (constrain) joint limits
% (constrain) torque limits

    controller      = obj.input_4_run{4};

    fall_penalty    = -1.5;  %in this case, I set a very negative penalty because in the unconstrained case i have no lower bound
    
    
    task_errors     = controller.simulation_results.task_errors;     %[nsamples x 12] matrix, [CoMx, CoMy, CoMz, OriRot, lFootx,lFooty,lFootz,lFootRot,rFootx, rFooty, rFootz, rFootRot]
%     joint_error     = controller.simulation_results.joint_error;     %[nsamples x nDOF]
    torques         = controller.simulation_results.torques;         %[nsamples x nDOF]
    QP_exitFlag      = controller.simulation_results.QP_exitFlag;      %[nsamples x 1]
%     zmp             = controller.simulation_results.zmp;             %[nsamples x 3]
%     support_polygon = controller.simulation_results.support_polygon; %[2 x 2 x nsamples]
%     feet_in_contact = controller.simulation_results.feet_in_contact; %[nsamples x 2]
%     pose_CoM        = controller.simulation_results.pose_CoM;        %[nsamples x 3]
%     pose_lFoot      = controller.simulation_results.pose_lFoot;      %[nsamples x 3]
%     pose_rFoot      = controller.simulation_results.pose_rFoot;      %[nsamples x 3]
    time            = controller.simulation_results.time;            %[nsamples x 1]
    zmpErr          = controller.simulation_results.zmpErr;            %[nsamples x 1]

    t_all           = output{1};
    q_all           = output{2};
    %qd_all          = output{3};
    
    nsamples        = size(controller.simulation_results.task_errors,1);
    %max_torques     = 230    * nsamples ; %! these parameters need to be scaled with the length of the simulation
    %max_task_error  = 0.1    * nsamples;
    %max_zmpErr      = 0.0049 * nsamples;
    
    max_torques     = 2000  * nsamples;
    max_task_error  = 4     * nsamples;
    max_zmpErr      = 0.005 * nsamples;

    weight_task_err = -1;      %minimize
    weight_torques  = -0.0001; %minimize
    weight_zmp_dist = -1;     %maximize (minimize a negative measure of distance)
    sum_weights     = abs(weight_torques) + abs(weight_task_err) + abs(weight_zmp_dist);
    
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
            res.torques(20),res.torques(20),res.torques(21),res.torques(21),res.torques(22),res.torques(22),res.torques(23),res.torques(23)};

        % here I update the value inside the penalty object
        obj.penalty_handling.EvaluateConstraints(input_vector,evaluate_constraints_index);
        evaluate_constraints_index = evaluate_constraints_index + 1;
    end

    %% for debug 
    % compute violation 
    %[constraints_violation_cost,penalties,violation_index] = ArtificialConstraintViolations(obj.penalty_handling.constraints_violation,obj.penalty_handling.n_constraint);
    %violation_index;
    %----


    %fitness function computation

    %sum of task errors
    sum_task_error = sum((task_errors(:).*task_errors(:)),1);
    
    %sum of joint torques
    sum_torques = sum((torques(:).*torques(:)),1);

    %sum of zmp error
    sum_zmpErr  = sum((zmpErr(:).*zmpErr(:)),1);

    % saturations
    if(sum_torques > max_torques)
        sum_torques = max_torques;
    end

    if(sum_task_error > max_task_error)
        sum_task_error = max_task_error;
    end
    
    if(sum_zmpErr > max_zmpErr)
        sum_zmpErr = max_zmpErr;
    end

    sum_task_error     = sum_task_error/max_task_error;
    sum_torques        = sum_torques/max_torques;
    sum_zmpErr         = sum_zmpErr/max_zmpErr;

    %Note: the optimization procedure searches to maximize the fitness
    fit = (weight_task_err * sum(sum_task_error) + weight_torques * sum_torques + weight_zmp_dist * sum_zmpErr) / sum_weights;

    failure = false;

    % check to see if the robot has fallen/was about to fall or the QP was unfeasible during the current rollout
    if length(time) < length(t_all)
        disp('for your info: robot has fallen');
        fit = fit  + fall_penalty;
        %failure = true;
    end
    % check to see if the QP could not be solved at any point during the current rollout
    if QP_exitFlag' * QP_exitFlag > 0
        disp('for your info: QP was not always successful');
        fit = fit + fall_penalty;
    end

    %%DEBUG
    fprintf('sum of task errors is %f\n', sum_task_error)
    fprintf('sum of torques is %f\n', sum_torques)
    fprintf('zmp error is %f\n', sum_zmpErr)
    fprintf('candidate fit is %f\n', fit)
    %---
        
    
end
    
