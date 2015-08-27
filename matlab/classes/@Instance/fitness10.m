

function fit = fitness10(obj,t,q)
    global G_OB;
    
    %%%;;
    downsaple = 10;
    L = 1; 
    max_effort = 1.5000e+06;
    max_traj_error = 5000;
    weight_effort = 1;
    weight_traj_err = 1;
    hitting_condition = 0.01;
    hitting = false;
    max_allowed_tau = 20;
    tau_violation = false;
    joint_violation = false; % joint in deg
    
    %%%EOF
    contr = obj.controller;
    traj_err= 0;
    

    for i=1:downsaple:size(t,2)
        q_cur = q{1}(i,:);
        % joint limits in deg for the second joint of jaco[47 313]
        if(q_cur(2)*(180/pi)<47 || q_cur(2)*(180/pi)>313)
           joint_violation = true;
        end
        % joint limits in deg for the third joint of jaco[47 313] [19 341]
        if(q_cur(2)*(180/pi)<47 || q_cur(2)*(180/pi)>341)
           joint_violation = true;
        end
      
        
        % compute the trajectory error (absolute error)
        kinematic6=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
        T6 = eval(kinematic6{1});
        ee = T6(1:3,4);
        attr_pos = contr.references.GetTraj(1,1,t(i)); 
        traj_err = traj_err + norm((ee - attr_pos),L);
        
        %compute position of the all control points
        kinematic5='contr.subchains.sub_chains{1}.T0_5(q_cur)';
        kinematic4='contr.subchains.sub_chains{1}.T0_4(q_cur)';
        kinematic3='contr.subchains.sub_chains{1}.T0_3(q_cur)';
        p5 = eval(kinematic5);
        p5 = p5(1:3,4);
        p4 = eval(kinematic4);
        p4 = p4(1:3,4);
        p3 = eval(kinematic3);
        p3 = p3(1:3,4);
        
        control_points =[ee, p5, p4, p3];
        % verify the pruning conditions
        for jj=1:size(G_OB,2)
            for jjj = 1 : size(control_points,2)
                dist = G_OB(jj).Dist(control_points(:,jjj)',L);
                if(dist < hitting_condition)
                   hitting = true;
                end
            end
        end
        if(hitting || joint_violation)
           break;
        end
    end
    
    % tau violation control
    for ii =1:size(contr.torques{1},2)
        for kk = 1:size(contr.torques{1},1)
            if(contr.torques{1}(kk,ii)>max_allowed_tau || contr.torques{1}(kk,ii)<-max_allowed_tau)
                tau_violation = true;
            end
        end
        if(tau_violation)
           break;
        end
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
    
    if(hitting)
      fit = -1;
      disp('hit the obstacle')
    end
    if(joint_violation)
      fit = -1;
      disp('joint violation')
    end
    if(tau_violation)
      fit = -1;
      disp('tau violation')
    end
end