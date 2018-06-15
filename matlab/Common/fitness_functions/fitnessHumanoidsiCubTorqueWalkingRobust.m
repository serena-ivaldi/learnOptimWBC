function [fit,failure]  = fitnessHumanoidsiCubTorqueWalkingRobust(obj,output)
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

    fall_penalty    = -1;  %in this case, I set a very negative penalty because in the unconstrained case i have no lower bound
    
    torques         = controller.simulation_results.torques;         %[nsamples x nDOF]
    time            = controller.simulation_results.time;            %[nsamples x 1]
    zmpErr          = controller.simulation_results.zmpErr;          %[nsamples x 1]
    QP_exitFlag     = controller.simulation_results.QP_exitFlag;     %[nsamples x 1]    

    t_all           = output{1};
    q_all           = output{2};
    qd_all          = output{3};
    
    nsamples        = size(controller.simulation_results.zmpErr,1);
    max_zmpErr      = 0.0049 * nsamples;
    
    
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
    
    %sum of zmperror
    sum_zmpErr  = sum((zmpErr(:).*zmpErr(:)),1);

    
    if(sum_zmpErr > max_zmpErr)
        sum_zmpErr = max_zmpErr;
    end

    sum_zmpErr         = sum_zmpErr/max_zmpErr;

    %Note: the optimization procedure searches to maximize the fitness
    fit =  sum_zmpErr;

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
    fprintf('candidate fit is %f\n', fit)
    %---
        
    
end
    
