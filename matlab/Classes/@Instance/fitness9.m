

function fit = fitness9(obj,t,q)
    global G_OB;
    
    %%%;;
    downsaple = 10;
    L = 1; 
    max_effort = 2000;
    max_traj_error = 5000;
    weight_effort = 1;
    weight_traj_err = 1;
    hitting_condition = 0.05;
    hitting = false;
    %%%EOF
    contr = obj.controller;
    traj_err= 0;
    

    for i=1:downsaple:size(t,2)
        q_cur = q{1}(i,:);
        % compute the trajectory error (absolute error)
        kinematic=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
        T = eval(kinematic{1});
        ee = T(1:3,4);
        attr_pos = contr.references.GetTraj(1,1,t(i)); 
        traj_err = traj_err + norm((ee - attr_pos),L);
        % verify the pruning conditions
        for jj=1:size(G_OB,2)
            dist = G_OB(jj).Dist(ee',L);
            if(dist < hitting_condition)
               hitting = true;
            end
        end
        if(hitting)
           break;
        end
    end
    % max effort
    effort = max(contr.torques{1}(:));
    
    
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
      fit = -2;
      disp('hit the obstacle')
    end
end