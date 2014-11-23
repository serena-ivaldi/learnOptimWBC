% this function return the current position and velocity of and effector 
% and for every subchain i recompute the jacobian (very idiot method TO FIX)
% in this functio we can compute the position using the current model (perturbed in general)
% and the ground truth model supposing that we perform the measure using 
% external sensors like a set of cameras


function [J,J_dot,x,xd,rpy,rpyd]=DirKin(obj,q,qd,ind_subchain,ind_task)

        q_cur = zeros(1,obj.sub_chains{ind_subchain}.n);
        qd_cur= zeros(1,obj.sub_chains{ind_subchain}.n);
        
        q_cur(1:obj.GetNumSubLinks(ind_subchain,ind_task)) = q(1:obj.GetNumSubLinks(ind_subchain,ind_task));
         qd_cur(1:obj.GetNumSubLinks(ind_subchain,ind_task)) = qd(1:obj.GetNumSubLinks(ind_subchain,ind_task));
        % compute pose (position + rool pitch yaw) from the current
        % subchain
        T = obj.sub_chains(ind_subchain).fkine(q_cur);
        x = T(1:3,4);
        rpy = tr2rpy(T);
        rpy = rpy';
        % compute generalized cartesian velocities from the current
        % subchain
        J = obj.sub_chains{ind_subchain}.jacob0(q_cur);
        v=J*qd_cur';
        xd = v(1:3);rpyd=v(4:6);
        % compute J_dot from the the current subchain
        J_dot = obj.sub_chains(index).jacob_dot(q_cur,qd_cur);   
  
end