function fit  = fitnessHumanoidsIcubStandUp(obj,output)
% fitness function of the two arms + manipulation experience on the iCub :
% reaching two goals behind a wall, one with each hands, before a fixed
% time Tswitch after which the goals positions behind the wall change and a
% fixed distance constraints activate. All that done with minimal torques
%and under the usual constraints : joints limits, torques limits and colision detection.
    
    t  = output{1};
    q  = output{2};
    qd = output{3};
     
    %%%;;
    downsaple = 1;
    L = 1; 
    max_effort = 3.5000e+05;
    max_traj_error = 250;
    weight_effort = 1;
    weight_traj_err = 3;
    %%%EOF
    param = obj.input_4_run{2};
    contr = obj.input_4_run{4};
    iCub = contr.GetWholeSystem();
    
    traj_err = 0;
    effort   = 0;
     
    evaluate_constraints_index = 1;
    for i=1:downsaple:size(t,1)
        t_cur = t(i);
        q_cur = q(i,:);
        qd_cur= qd(i,:);
        chi_cur = [q_cur,qd_cur];
        q = q_cur(8:8+iCub.ndof);
        [dchi,res]=forwardDynamics(t_cur,chi_cur,contr,param);
        
        %% fitness computation
        traj_err = traj_err + norm((params.xComfinal - res.xCoM),L);
        effort   = effort   + norm(res.tau);
        %% constraint computation
        
        
        balance      = CheckBalance(res.zmp,iCub.support_poly);
        
        
        input_vector = {q(1),q(1),q(2),q(2),q(3),q(3),q(4),q(4),q(5),q(5),q(6),q(6),q(7),rq(7),q(8),q(8),q(9),q(9),q(10),q(10),q(11),q(11),q(12),q(12),q(13),q(13),q(14),q(14),...
                        q(15),q(15),q(16),q(16),q(17),q(17),q(18),q(18),q(19),q(19),q(20),q(20),q(21),q(21),q(22),q(22),q(23),q(23),q(24),q(24),q(25),q(25),...
                        res.tau(1),res.tau(1),res.tau(2),res.tau(2),res.tau(3),res.tau(3),res.tau(4),res.tau(4),res.tau(5),res.tau(5),res.tau(6),res.tau(6),res.tau(7),res.tau(7),...
                        res.tau(8),res.tau(8),res.tau(9),res.tau(9),res.tau(10),res.tau(10),res.tau(11),res.tau(11),res.tau(12),res.tau(12),res.tau(13),res.tau(13),res.tau(14),res.tau(14),...
                        res.tau(15),res.tau(15),res.tau(16),res.tau(16),res.tau(17),res.tau(17),res.tau(18),res.tau(18),res.tau(19),res.tau(19),res.tau(20),res.tau(20),res.tau(21),res.tau(21),...
                        res.tau(22),res.tau(22),res.tau(23),res.tau(23),res.tau(24),res.tau(24),res.tau(25),res.tau(25),...
                        balance};
                                

        % here i update the value inside the penalty object       
        obj.penalty_handling.EvaluateConstraints(input_vector,evaluate_constraints_index);
        evaluate_constraints_index = evaluate_constraints_index + 1;   
    end 
    % saturations 
    if(effort>max_effort)
       effort = max_effort;
    end

    if(traj_err>max_traj_error)
       traj_err = max_traj_error;
    end

    traj_err  = traj_err/max_traj_error;
    effort = effort/max_effort;
    %%DEBUG
    fprintf('traj error is %f\n', traj_err)
    fprintf('effort term is %f\n', effort)
    %---
    fit = (-traj_err*(weight_traj_err) - effort*(weight_effort))*( 1/(weight_traj_err+weight_effort) );
end
