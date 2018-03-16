function [fit,failure]  = fitnessHumanoidsIcubStandUpOptSolutionNoBackwardSimulink(obj,output)
% fitness function of the icub standing up we try to optimize the cost
% of consumption and the error with the final com (for constrained optimization)
% here i consider one more features: i penalize the solution if i have
% backward movement of the com
    
    t_all  = output{1};
    q_all  = output{2};
    qd_all = output{3};
     
    %%%;;
    downsaple           = 1;
    L                   = 1; 
    fall_penalty        = -1; %  in this case i set a very negative penalty because in the uncostrained case i have no lower bound 
    max_effort          = 3.1000e+05;
    max_traj_error      = 100;
    max_backward_err    =  0.3700;
    weight_effort       = 1;
    weight_traj_err     = 1.3;
    weight_backward_err = 3;
    %%%EOF
    param = obj.input_4_run{2};
    contr = obj.input_4_run{4};
    iCub = contr.GetWholeSystem();
    
    % for trajectory
    traj_err= 0;
    backward_err = 0;
     
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
    
    %% robot fault (falling robot)
    if(leftCount > 0 || RightCount > 0)
        disp('robot has fallen')
        fprintf('constraints violation is %f\n', fall_penalty)
        fit = fall_penalty;
        failure = true;
    %% no fault (robot reached the final position)    
    else    
        for i=1:downsaple:length(t_all)

            res.tau  = contr.simulation_results.tau(i,:);
            q        = q_all(i,:);%contr.simulation_results.q(i,:);
            res.xCoM = contr.simulation_results.xCoM(i,:);
            res.zmp  = contr.simulation_results.zmp(i,:);

            
            %% fitness function computation 
             
            % com error 
            traj_err = traj_err + norm((res.xCoM - param.final_com),L);
            % backward error 
            if(i>1)
                % backward movements on x
                if(previous_com(1)<res.xCoM(1))
                    backward_err = backward_err + abs(res.xCoM(1) - previous_com(1));
                end
                % backward movements on z
                if(previous_com(3)<res.xCoM(3))
                    backward_err = backward_err + abs(res.xCoM(3) - previous_com(3));
                end
                    
            end
            
            
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
            % i save the current com to compare to the next one
            previous_com = res.xCoM;
        end 
        
        %% for debug 
        % compute violation 
        [constraints_violation_cost,penalties,violation_index] = ArtificialConstraintViolations(obj.penalty_handling.constraints_violation,obj.penalty_handling.n_constraint);
        violation_index
        %----
        
        % max effort
        transpose_torque = contr.simulation_results.tau';
        effort = sum((transpose_torque(:).*transpose_torque(:)),1);
        %effort = sum(effort,1);
        % saturations 
        if(effort>max_effort)
           effort = max_effort;
        end

        if(traj_err>max_traj_error)
           traj_err = max_traj_error;
        end
        
        if(backward_err > max_backward_err)
            backward_err = max_backward_err;
        end

        traj_err     = traj_err/max_traj_error;
        effort       = effort/max_effort;
        backward_err = backward_err/max_backward_err;
        fit = (-traj_err*(weight_traj_err) -effort*(weight_effort) -backward_err*(weight_backward_err) )*( 1/(weight_traj_err + weight_effort + weight_backward_err) );
        %%DEBUG
        fprintf('traj error is %f\n', traj_err)
        fprintf('effort term is %f\n', effort)
        fprintf('backward err is %f\n', backward_err)
        fprintf('candidate fit is %f\n', fit)
        %---
     
        failure = false; 
    end
end
