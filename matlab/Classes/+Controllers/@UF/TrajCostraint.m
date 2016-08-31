% in this function i compute a part of control action giving the desidered
% position and velocity and comparing them with the actual position and
% velocity plus i extract from the jacobian the task jacobian and J_dot task  

function [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J_old,Jd_old,x,xd,rpy,rpyd,q,qd,Fc)
    tot_link=obj.subchains.GetNumLinks(ind_subchain);
    sub_link=obj.subchains.GetNumSubLinks(ind_subchain,ind_task);
    if(strcmp(obj.references.type{ind_subchain,ind_task},'joint'))
         % in this part of the code i dont need to use reshape jacobian
         % because i will use for use the complete jacobian for control in joint
         % space
         subchain = obj.subchains.sub_chains{1};
         if ( ~isa(subchain, 'DummyRvc_iCub') && ~isa(subchain, 'WBM.Interfaces.IMultChainTree') ) 
            A=eye(obj.GetActiveBot.n);
         else
             if (obj.GetWholeSystem.active_floating_base)
                A=[zeros(32,6), eye(obj.subchains.whole_system.ndof)];
             else
                A=eye(obj.subchains.whole_system.ndof); 
             end
         end
         [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
         b = PD(q,x_des,obj.Param{ind_subchain,ind_task}.Kp,qd,xd_des,obj.Param{ind_subchain,ind_task}.Kd,xdd_des);
         % J_dot is not used anymore because im imposing a trajecotry in
         % the joint
         return;
    elseif(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian'))
         if(strcmp(obj.references.control_type{ind_subchain,ind_task},'x'))
             x_cur  = x;
             xd_cur = xd;
             jacob_type = 'trans';   
         elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'rpy'))
             x_cur  = rpy;
             xd_cur = rpyd;    
             jacob_type = 'rot';
         else
             error('wrong specification of the control_type. it should be x or rpy');
         end
         [A,J_dot,~] = ReshapeJacobian(J_old,Jd_old,Fc,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},jacob_type);
         [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
         b = PD(x_cur,x_des,obj.Param{ind_subchain,ind_task}.Kp,xd_cur,xd_des,obj.Param{ind_subchain,ind_task}.Kd,xdd_des);
         % J_dot is just multiplied by qd
         b = b - J_dot;
         return;     
    elseif(strcmp(obj.references.type{ind_subchain,ind_task},'impedance'))
         if(strcmp(obj.references.control_type{ind_subchain,ind_task},'x'))
             x_cur  = x;
             xd_cur = xd;
             jacob_type = 'trans';   
         elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'rpy'))
             x_cur  = rpy;
             xd_cur = rpyd;    
             jacob_type = 'rot';
        end
        [J,J_dot,Fc] = ReshapeJacobian(J_old,Jd_old,Fc,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},jacob_type);
        [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
        % obj.Param{1} = M
        % obj.Param{2} = D
        % obj.Param{3} = P
        A = obj.Param{ind_subchain,ind_task}.M*J;
        b = Fc - obj.Param{ind_subchain,ind_task}.M*xdd_des - obj.Param{ind_subchain,ind_task}.D*(xd_cur - xd_des)...
            - obj.Param{ind_subchain,ind_task}.P*(x_cur-x_des) - obj.Param{ind_subchain,ind_task}.M*J_dot;
        %% DEBUG
        %errorx_impedance  = norm(x_des - x_cur)
        %errorxd_impedance = norm(xd_des - xd_cur)
        %%---
    end
end
