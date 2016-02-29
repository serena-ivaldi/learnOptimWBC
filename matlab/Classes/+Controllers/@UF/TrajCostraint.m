% in this function i compute a part of control action giving the desidered
% position and velocity and comparing them with the actual position and
% velocity plus i extract from the jacobian the task jacobian and J_dot task  

function [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J_old,Jd_old,x,xd,rpy,rpyd,q,qd,Fc)
 tot_link=obj.subchains.GetNumLinks(ind_subchain);
 sub_link=obj.subchains.GetNumSubLinks(ind_subchain,ind_task);
 if(strcmp(obj.references.type{ind_subchain,ind_task},'joint'))
      if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))
            % in this part of the code i dont need to use reshape jacobian
            % because i will use for use the complete jacobian for control in joint
            % space
            A=eye(obj.GetActiveBot.n);
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(q,x_des,obj.Kp{ind_subchain,ind_task},qd,xd_des,obj.Kd{ind_subchain,ind_task},xdd_des);
            % J_dot is not used anymore because im imposing a trajecotry in
            % the joint
            return;
        elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
        % tracking in the joint space is not usefull for now
      end
 elseif(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian_x'))
     if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'trans');
         [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
         b = PD(x,x_des,obj.Kp{ind_subchain,ind_task},xd,xd_des,obj.Kd{ind_subchain,ind_task},xdd_des);
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;
     elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'trans');
         [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
         b = PD(x,x_des,obj.Kp{ind_subchain,ind_task},xd,xd_des,obj.Kd{ind_subchain,ind_task},xdd_des);  
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;
     elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'impedance')) 
        [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'trans');
        [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
        % obj.Param{1} = M
        % obj.Param{2} = D
        % obj.Param{3} = P
        A = J'*obj.Param{1};
        b = Fc - obj.Param{1}*xdd_des - obj.Param{2}(J_dot - xd_des) - obj.Param{3}*(x-x_des) - obj.Param{1}*J_dot;
     end
 elseif(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian_rpy'))  
     if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation')) 
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'rot');
         [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
         b = PD(rpy,rpy_des,obj.Kp{ind_subchain,ind_task},rpyd,rpyd_des,obj.Kd{ind_subchain,ind_task},rpydd_des);  
         % J_dot is just multiplied by qd
         b = b - J_dot; 
         return;
     elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
         [A,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'rot');
         [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
         b = PD(rpy,rpy_des,obj.Kp{ind_subchain,ind_task},rpyd,rpyd_des,obj.Kd{ind_subchain,ind_task},rpydd_des);  
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;
     elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'impedance'))
        [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'trans');
         [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
        % obj.Param{1} = M
        % obj.Param{2} = D
        % obj.Param{3} = P
        A = J'*obj.Param{1};
        b = Fc - obj.Param{1}*rpydd_des - obj.Param{2}(J_dot - rpyd_des) - obj.Param{3}*(x-rpy_des) - obj.Param{1}*J_dot;
     end 
 end
end