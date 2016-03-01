% in this function i compute a part of control action giving the desidered
% position and velocity and comparing them with the actual position and
% velocity plus i extract from the jacobian the task jacobian and J_dot task  

function [b,A] = TrajCostraintSecondary(obj,ind_subchain,ind_task,t,J_old,Jd_old,x,xd,rpy,rpyd,q,qd,Fc)
 tot_link=obj.subchains.GetNumLinks(ind_subchain);
 sub_link=obj.subchains.GetNumSubLinks(ind_subchain,ind_task);
 if(strcmp(obj.Secondary_refs.type{ind_subchain,ind_task},'joint'))
      if(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'regulation'))
            % in this part of the code i dont need to use reshape jacobian
            % because i will use for use the complete jacobian for control in joint
            % space
            A=eye(obj.GetActiveBot.n);
            [x_des,xd_des,xdd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
            b = PD(q,x_des,obj.Param_secondary{ind_subchain,ind_task}.Kp,qd,xd_des,obj.Param_secondary{ind_subchain,ind_task}.Kd,xdd_des);
            % J_dot is not used anymore because im imposing a trajecotry in
            % the joint
            return;
        elseif(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'tracking'))
        % tracking in the joint space is not usefull for now
      end
 elseif(strcmp(obj.Secondary_refs.type{ind_subchain,ind_task},'cartesian_x'))
     if(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'regulation'))
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.Secondary_refs.mask{ind_subchain,ind_task},'trans');
         [x_des,xd_des,xdd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
         b = PD(x,x_des,obj.Param_secondary{ind_subchain,ind_task}.Kp,xd,xd_des,obj.Param_secondary{ind_subchain,ind_task}.Kd,xdd_des);
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;
     elseif(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'tracking'))
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.Secondary_refs.mask{ind_subchain,ind_task},'trans');
         [x_des,xd_des,xdd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
         b = PD(x,x_des,obj.Param_secondary{ind_subchain,ind_task}.Kp,xd,xd_des,obj.Param_secondary{ind_subchain,ind_task}.Kd,xdd_des);  
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;
     elseif(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'impedance')) 
        [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.Secondary_refs.mask{ind_subchain,ind_task},'trans');
        [x_des,xd_des,xdd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
        % obj.Param{1} = M
        % obj.Param{2} = D
        % obj.Param{3} = P
        A = J'*obj.Param_secondary{ind_subchain,ind_task}.M;
        b = Fc - obj.Param_secondary{ind_subchain,ind_task}.M*xdd_des - obj.Param_secondary{ind_subchain,ind_task}.D*(J_dot - xd_des) ...
            - obj.Param_secondary{ind_subchain,ind_task}.P*(x-x_des) - obj.Param_secondary{ind_subchain,ind_task}.D*J_dot;
     end
 elseif(strcmp(obj.Secondary_refs.type{ind_subchain,ind_task},'cartesian_rpy'))  
     if(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'regulation')) 
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.Secondary_refs.mask{ind_subchain,ind_task},'rot');
         [rpy_des,rpyd_des,rpydd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
         b = PD(rpy,rpy_des,obj.Param_secondary{ind_subchain,ind_task}.Kp,rpyd,rpyd_des,obj.Param_secondary{ind_subchain,ind_task}.Kd,rpydd_des);  
         % J_dot is just multiplied by qd
         b = b - J_dot; 
         return;
     elseif(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'tracking'))
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.Secondary_refs.mask{ind_subchain,ind_task},'rot');
         [rpy_des,rpyd_des,rpydd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
         b = PD(rpy,rpy_des,obj.Param_secondary{ind_subchain,ind_task}.Kp,rpyd,rpyd_des,obj.Param_secondary{ind_subchain,ind_task}.Kp,rpydd_des);  
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;
     elseif(strcmp(obj.Secondary_refs.control_type{ind_subchain,ind_task},'impedance'))
        [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.Secondary_refs.mask{ind_subchain,ind_task},'trans');
         [rpy_des,rpyd_des,rpydd_des] = obj.Secondary_refs.GetTraj(ind_subchain,ind_task,t);
        % obj.Param{1} = M
        % obj.Param{2} = D
        % obj.Param{3} = P
        A = J'*obj.Param_secondary{ind_subchain,ind_task}.M;
        b = Fc - obj.Param_secondary{ind_subchain,ind_task}.M*rpydd_des - obj.Param_secondary{ind_subchain,ind_task}.D*(J_dot - rpyd_des)...
            - obj.Param_secondary_secondary{ind_subchain,ind_task}.P*(x-rpy_des) - obj.Param_secondary_secondary{ind_subchain,ind_task}.M*J_dot;
     end 
 end
end