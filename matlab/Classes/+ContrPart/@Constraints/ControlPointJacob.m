% this function compute the jacobian of the control point for obstacle if
% there is no task associated with the control point

function [J ,cp]=ControlPointJacob(obj,index_control_point,q)


        cur_bot = obj.bot;

        q_cur = zeros(1,cur_bot.n);
        q_cur(1:index_control_point) = q(1:index_control_point);
        kinematic=CStrCatStr({'cur_bot.T0_'},num2str(index_control_point),{'(q_cur)'});
        T = eval(kinematic{1});
        cp = T(1:3,4);
     
        % compute generalized cartesian velocities from the current
        % subchain
        J = cur_bot.jacob0(q_cur);
        % i am interested only in the position block of the jacobian
        J = J(1:3,:);
        
        
         
  
end