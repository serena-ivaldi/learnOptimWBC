% this function return the current position and velocity of and effector 
% and for every subchain i recompute the jacobian (very idiot method TO FIX)
% in this functio we can compute the position using the current model (perturbed in general)
% and the ground truth model supposing that we perform the measure using 
% external sensors like a set of cameras


function [J,J_dot,x,xd,rpy,rpyd]=DirKin(obj,index,q,qd,ground_truth)



    if(ground_truth)
       
        % compute pose (position + rool pitch yaw) from the GT subchain
        T = obj.sub_chainsGT(index).fkine(q(1:obj.GetNumSubLinks(index)));
        x = T(1:3,4);
        rpy = tr2rpy(T);
        rpy = rpy';
        % compute generalized cartesian velocities from the GT subchain
        J_GT=obj.sub_chainsGT(index).jacob0(q(1:obj.GetNumSubLinks(index)));
        v=J_GT*qd(1:obj.GetNumSubLinks(index));
        xd = v(1:3);rpyd=v(4:6);
        % compute jacobian and J_dot from the pertubed subchain
        J = obj.sub_chains(index).jacob0(q(1:obj.GetNumSubLinks(index)));
        J_dot = obj.sub_chains(index).jacob_dot(q(1:obj.GetNumSubLinks(index)),qd(1:obj.GetNumSubLinks(index)));
    
    else
        
        % compute pose (position + rool pitch yaw) from the pertubed
        % subchain
        T = obj.sub_chains(index).fkine(q(1:obj.GetNumSubLinks(index)));
        x = T(1:3,4);
        rpy = tr2rpy(T);
        rpy = rpy';
        % compute generalized cartesian velocities from the pertubed
        % subchain
        J = obj.sub_chains(index).jacob0(q(1:obj.GetNumSubLinks(index)));
        v=J*qd(1:obj.GetNumSubLinks(index))';
        xd = v(1:3);rpyd=v(4:6);
        % compute J_dot from the the pertubed subchain
        J_dot = obj.sub_chains(index).jacob_dot(q(1:obj.GetNumSubLinks(index)),qd(1:obj.GetNumSubLinks(index)));   
    end    





end