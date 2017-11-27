% this function return the current position and velocity of and effector
% and for every subchain i recompute the jacobian (very idiot method TO FIX)
% in this functio we can compute the position using the current model (perturbed in general)
% and the ground truth model supposing that we perform the measure using
% external sensors like a set of cameras


function [J,J_dot,x,xd,rpy,rpyd]=DirKin(obj,q,qd,ind_subchain,ind_task)


        cur_bot = obj.GetCurRobot(ind_subchain);
        % i have to distinguish between icub and rbt robots (hopefully provisory)
        if(isa(cur_bot,'DummyRvc_iCub'))
            if ~(strcmp(obj.target_link{ind_subchain}{ind_subchain,ind_task},'none'))
                % UPDATE OF THE TAG VALUE INSIDE DUMMY_ROBOT
                cur_bot.tag = obj.target_link{ind_subchain}{ind_subchain,ind_task};
                % compute pose (position + rool pitch yaw) from the current
                % subchain
                T = cur_bot.fkine(q');
                x = T(1:3,4);
                rpy = tr2rpy(T);
                rpy = rpy';
            else
                x = [];
                rpy = [];
            end
            %try
            if (obj.whole_system.active_floating_base)
                J = cur_bot.jacob0(q','rpy');
                full_qd = [obj.whole_system.dx_b;obj.whole_system.omega_W;qd'];
                v=J*full_qd;
                xd = v(1:3);rpyd=v(4:6);
            else
                J = cur_bot.jacob0(q','rpy');
                J = J(:,7:end);
                v=J*qd';
                xd = v(1:3);rpyd=v(4:6);
            end
            %catch error
            %  error('jacob0 is symbolic. remove the corresponding mex file in the robot folder to make it works','could be a representation singularity');
            %end
            % compute J_dot from the the current subchain
            J_dot = cur_bot.jacob_dot(q',qd');
        elseif isa(cur_bot, 'WBM.Interfaces.IMultChainTree')
            % 'cur_bot' is a derived class of the 'IMultChainTree' interface:
            q_t = q.';
            dq_t = qd.';
            lnk_name = obj.target_link{ind_subchain}{ind_subchain, ind_task};

            if ~strcmp(lnk_name, 'none')
                % change the current link of the mult-chain tree robot
                % which is controlled by the system ...
                cur_bot.link = lnk_name;

                % compute the pose (ZYX-position) of the current sub-chain ...
                fk_tform = cur_bot.fkine(q_t);
                x   = fk_tform(1:3,4);
                rpy = WBM.utilities.tfms.tform2eul(fk_tform, 'ZYX');
            else
                x   = [];
                rpy = [];
            end

            J = cur_bot.jacob0(q_t, 'rpy');
            if (obj.whole_system.active_floating_base)
                v_full = vertcat(obj.whole_system.dx_b, obj.whole_system.omega_W, dq_t);
                dv     = J*v_full;

                xd   = dv(1:3,1);
                rpyd = dv(4:6,1);
            else
                J  = J(:,7:end);
                dv = J*dq_t;

                xd   = dv(1:3,1);
                rpyd = dv(4:6,1);
            end

            % compute the the derivative Jacobian of the current sub-chain ...
            J_dot = cur_bot.jacob_dot(q_t, dq_t);
        elseif(isa(cur_bot,'SerialLink'))
            % TODO this part has to be removed it those not make any sense
            q_cur = zeros(1,obj.sub_chains{ind_subchain}.n);
            qd_cur= zeros(1,obj.sub_chains{ind_subchain}.n);
            q_cur(1:obj.GetNumSubLinks(ind_subchain,ind_task)) = q(1:obj.GetNumSubLinks(ind_subchain,ind_task));
            qd_cur(1:obj.GetNumSubLinks(ind_subchain,ind_task)) = qd(1:obj.GetNumSubLinks(ind_subchain,ind_task));
            % compute pose (position + rool pitch yaw) from the current
            % subchain
            kinematic=CStrCatStr({'cur_bot.T0_'},num2str(obj.GetNumSubLinks(ind_subchain,ind_task)),{'(q_cur)'});
            T = eval(kinematic{1});
            %T=cur_bot.T0_6(q_cur)
            %T = cur_bot.fkine(q_cur);
            x = T(1:3,4);
            rpy = tr2rpy(T);
            rpy = rpy';

            % i ahve to provide a common way to manage the jacobian
            %try
            J = cur_bot.jacob0(q_cur,'rpy');
            %catch error
            %  error('jacob0 is symbolic. remove the corresponding mex file in the robot folder to make it works','could be a representation singularity');
            %end
            v=J*qd_cur';
            xd = v(1:3);rpyd=v(4:6);
            % compute J_dot from the the current subchain
            J_dot = cur_bot.jacob_dot(q_cur,qd_cur);
        end
%         % compute generalized cartesian velocities from the current
%         % subchain
%         %try
%           J = cur_bot.jacob0(q_cur,'rpy');
%         %catch error
%         %  error('jacob0 is symbolic. remove the corresponding mex file in the robot folder to make it works','could be a representation singularity');
%         %end
%
%         v=J*qd_cur';
%         xd = v(1:3);rpyd=v(4:6);
%         % compute J_dot from the the current subchain
%         J_dot = cur_bot.jacob_dot(q_cur,qd_cur);

end
