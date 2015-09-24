function [x]=CartPos(obj,q,ind_subchain,ind_task)

        q_cur = zeros(1,obj.sub_chains{ind_subchain}.n);
        q_cur(1:obj.GetNumSubLinks(ind_subchain,ind_task)) = q(1:obj.GetNumSubLinks(ind_subchain,ind_task));
       
        cur_bot = obj.GetCurRobot(ind_subchain);
        % compute pose (position + rool pitch yaw) from the current
        % subchain
        kinematic=CStrCatStr({'cur_bot.T0_'},num2str(obj.GetNumSubLinks(ind_subchain,ind_task)),{'(q_cur)'});
        T = eval(kinematic{1});
        %T=cur_bot.T0_6(q_cur)
        %T = cur_bot.fkine(q_cur);
        x = T(1:3,4);       
end