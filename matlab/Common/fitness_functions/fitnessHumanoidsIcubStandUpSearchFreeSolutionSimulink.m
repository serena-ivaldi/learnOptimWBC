function fit  = fitnessHumanoidsIcubStandUpSearchFreeSolutionSimulink(obj,output)
% fitness function of the icub standing up we try to reduce to zero the
% constraints violations to find a free solution for the constrained
% optimization (this fitness is an unconstrained problem)
    
    t_all  = output{1};
    q_all  = output{2};
    qd_all = output{3};
     
    %%%;;
    downsaple = 1;
    L = 1; 
    fall_penalty = -10000; %  in this case i set a very negative penalty because in the uncostrained case i have no lower bound 
    %%%EOF
    param = obj.input_4_run{2};
    contr = obj.input_4_run{4};
    iCub = contr.GetWholeSystem();
    
     
    evaluate_constraints_index = 1;
    
    %% check to see if the robot has fallen during the current rollout
    switching_time = param.tswitch;
    % get all the data samples after the switching_time (robot on its feet)
    index                               = find(contr.simulation_results.LsoleWrench.time==switching_time);
    final_index                         = find(contr.simulation_results.LsoleWrench.time==param.tEnd);
    reduced_left_leg_wrench_sim         = getdatasamples(contr.simulation_results.LsoleWrench,index:final_index);
    index                               = find(contr.simulation_results.RsoleWrench.time==switching_time);
    final_index                         = find(contr.simulation_results.RsoleWrench.time==param.tEnd);
    reduced_right_leg_wrench_sim        = getdatasamples(contr.simulation_results.RsoleWrench,index:final_index); 
    % check zero crossing for the f_z (no weight on the feet == robot fallen)
    leftCount = ZeroCrossingCount(reduced_left_leg_wrench_sim(:,3));
    RightCount = ZeroCrossingCount(reduced_right_leg_wrench_sim(:,3));
    
    if(leftCount > 0 || RightCount > 0)
        disp('robot has fallen')
        fprintf('constraints violation is %f\n', fall_penalty)
        fit = fall_penalty;
    else    
        for i=1:downsaple:length(t_all)

            res.tau  = contr.simulation_results.tau(i,:);
            q        = q_all(i,:);%contr.simulation_results.q(i,:);
            res.xCoM = contr.simulation_results.xCoM(i,:);
            res.zmp  = contr.simulation_results.zmp(i,:);

            %% constraint computation
            balance      = CheckBalance(res.zmp,iCub.support_poly);
            % in this way i assure that the zmp value before the switch are
            % ignored (they are not meaningfull)
            if(t_all(i)<=switching_time)
                balance = -0.1;
            end

            input_vector = {q(1),q(1),q(2),q(2),q(3),q(3),q(4),q(4),q(5),q(5),q(6),q(6),q(7),q(7),q(8),q(8),q(9),q(9),q(10),q(10),q(11),q(11),q(12),q(12),...
                            q(13),q(13),q(14),q(14),q(15),q(15),q(16),q(16),q(17),q(17),q(18),q(18),q(19),q(19),q(20),q(20),q(21),q(21),q(22),q(22),q(23),q(23),...
                            res.tau(1),res.tau(1),res.tau(2),res.tau(2),res.tau(3),res.tau(3),res.tau(4),res.tau(4),res.tau(5),res.tau(5),res.tau(6),res.tau(6),res.tau(7),res.tau(7),...
                            res.tau(8),res.tau(8),res.tau(9),res.tau(9),res.tau(10),res.tau(10),res.tau(11),res.tau(11),res.tau(12),res.tau(12),...
                            res.tau(13),res.tau(13),res.tau(14),res.tau(14),res.tau(15),res.tau(15),res.tau(16),res.tau(16),res.tau(17),res.tau(17),res.tau(18),res.tau(18),res.tau(19),res.tau(19),...
                            res.tau(20),res.tau(20),res.tau(21),res.tau(21),res.tau(22),res.tau(22),res.tau(23),res.tau(23),...
                            balance};


            % here i update the value inside the penalty object       
            obj.penalty_handling.EvaluateConstraints(input_vector,evaluate_constraints_index);
            evaluate_constraints_index = evaluate_constraints_index + 1;   
        end 
        % compute violation 
        [constraints_violation_cost,penalties,violation_index] = ArtificialConstraintViolations(obj.penalty_handling.constraints_violation,obj.penalty_handling.n_constraint);
        
        %%DEBUG
        disp('violation index')
        violation_index
        fprintf('constraints violation is %f\n', constraints_violation_cost)
        %---
        fit = -constraints_violation_cost;
    end
end
