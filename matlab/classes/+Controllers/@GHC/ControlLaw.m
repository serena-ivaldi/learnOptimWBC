% in this function i compute a part of control action giving the desidered
% position and velocity and comparing them with the actual position and
% velocity plus i extract from the jacobian the task jacobian and J_dot task  

function [b,J,J_dot] = ControlLaw(obj,ind_subchain,ind_task,t,J_old,Jd_old,x,xd,rpy,rpyd,q,qd)

 tot_link=obj.subchains.GetNumLinks(ind_subchain);
 sub_link=obj.subchains.GetNumSubLinks(ind_subchain,ind_task);

 if(strcmp(obj.references.type{ind_subchain,ind_task},'joint'))
     
      if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))

            % in this part of the code i dont need to use reshape jacobian
            % because i will use for use the complete jacobian for control in joint
            % space
            J=J_old;
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(q,x_des,obj.Kp{ind_subchain}(:,:,ind_task),qd,xd_des,obj.Kd{ind_subchain}(:,:,ind_task),xdd_des);

            return;

        elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
        % tracking in the joint space is not usefull for now
      end
 else
        
    if(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian_x'))
    
        if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))

            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'trans');
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(x,x_des,obj.Kp{ind_subchain}(:,:,ind_task),xd,xd_des,obj.Kd{ind_subchain}(:,:,ind_task),xdd_des);

            return;

        elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
           
            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'trans');
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(x,x_des,obj.Kp{ind_subchain}(:,:,ind_task),xd,xd_des,obj.Kd{ind_subchain}(:,:,ind_task),xdd_des);  

            return;
        end

    elseif(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian_rpy'))  
        
        if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))
            
            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'rot');
            [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(rpy,rpy_des,obj.Kp{ind_subchain}(:,:,ind_task),rpyd,rpyd_des,obj.Kd{ind_subchain}(:,:,ind_task),rpydd_des);  

            
        elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
            
            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,tot_link,sub_link,obj.references.mask{ind_subchain,ind_task},'rot');
            [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(rpy,rpy_des,obj.Kp{ind_subchain}(:,:,ind_task),rpyd,rpyd_des,obj.Kd{ind_subchain}(:,:,ind_task),rpydd_des);     
            
        end
        
    end    
         
 end
            
            
end            