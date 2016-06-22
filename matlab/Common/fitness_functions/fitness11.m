function fit  = fitness11(obj,output) 
    
    t = output{1};
    q = output{2};
    
    %%%;;
    downsaple = 10;
    L = 1; 
    max_effort = 1.0000e+06;
    max_traj_error = 4000;
    weight_effort = 1;
    weight_traj_err = 1;
    %%%EOF
    contr = obj.input_4_run{5};
    traj_err= 0;
    
    % i have to uniform the tau with the number of q
    tau_=InterpTorque(contr,obj.input_4_run{4},0.001);
    evaluate_constraints_index = 1;
    for i=1:downsaple:size(t,2)
        q_cur = q{1}(i,:);
        tau_cur = tau_(i,:);
        %compute position of the all control points
        kinematic6=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
        kinematic5='contr.subchains.sub_chains{1}.T0_5(q_cur)';
        kinematic4='contr.subchains.sub_chains{1}.T0_4(q_cur)';
        kinematic3='contr.subchains.sub_chains{1}.T0_3(q_cur)';
        ee = eval(kinematic6{1});
        ee = ee(1:3,4);
        p5 = eval(kinematic5);
        p5 = p5(1:3,4);
        p4 = eval(kinematic4);
        p4 = p4(1:3,4);
        p3 = eval(kinematic3);
        p3 = p3(1:3,4);
        control_points =[ee'; p5'; p4'; p3'];
        % here i build the input vector to compute the constraints
        % violations
        input_vector = {q_cur(2),q_cur(3),q_cur(2),q_cur(3),tau_cur(1),tau_cur(1),tau_cur(2),tau_cur(2),tau_cur(3),tau_cur(3),...
                       tau_cur(4),tau_cur(4),tau_cur(5),tau_cur(5),tau_cur(6),tau_cur(6),control_points(1,:),control_points(2,:),control_points(3,:),control_points(4,:)};
        
        % here i update the value inside the penalty        
        obj.penalty_handling.EvaluateConstraints(input_vector,evaluate_constraints_index);
        evaluate_constraints_index = evaluate_constraints_index + 1;
        
        % cartesian position error 
        attr_pos = contr.references.GetTraj(1,1,t(i)); 
        traj_err = traj_err + norm((ee - attr_pos),L);
    end 
    % max effort
    effort = sum((contr.torques{1}(:).*contr.torques{1}(:)),2);
    effort = sum(effort,1);
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