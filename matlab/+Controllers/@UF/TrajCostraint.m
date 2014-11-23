% in this function i compute a part of control action giving the desidered
% position and velocity and comparing them with the actual position and
% velocity plus i extract from the jacobian the task jacobian and J_dot task  

function [b,J] = TrajCostraint(obj,ind_subchain,ind_task,t,J_old,Jd_old,x,xd,rpy,rpyd)

 if(strcmp(obj.references.type{ind_subchain,ind_task},'joint'))
 
 else
        
    if(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian_x'))
    
        if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))

            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,obj.references.mask{ind_subchain,ind_task},'trans');
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(x,x_des,obj.Kp{ind_subchain}(:,:,ind_task),xd,xd_des,obj.Kd{ind_subchain}(:,:,ind_task),xdd_des);

            % J_dot is just multiplied by qd
            b = b - J_dot;

            return;

        elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
           
            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,obj.references.mask{ind_subchain,ind_task},'trans');
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(x,x_des,obj.Kp{ind_subchain}(:,:,ind_task),xd,xd_des,obj.Kd{ind_subchain}(:,:,ind_task),xdd_des);  

            % J_dot is just multiplied by qd
            b = b - J_dot;

            return;
        end

    elseif(strcmp(obj.references.type{ind_subchain,ind_task},'cartesian_rpy'))  
        
        if(strcmp(obj.references.control_type{ind_subchain,ind_task},'regulation'))
            
            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,obj.references.mask{ind_subchain,ind_task},'rot');
            [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(rpy,rpy_des,obj.Kp{ind_subchain}(:,:,ind_task),rpyd,rpyd_des,obj.Kd{ind_subchain}(:,:,ind_task),rpydd_des);  

            % J_dot is just multiplied by qd
            b = b - J_dot; 
            
        elseif(strcmp(obj.references.control_type{ind_subchain,ind_task},'tracking'))
            
            [J,J_dot] = ReshapeJacobian(J_old,Jd_old,obj.references.mask{ind_subchain,ind_task},'rot');
            [rpy_des,rpyd_des,rpydd_des] = obj.references.GetTraj(ind_subchain,ind_task,t);
            b = PD(rpy,rpy_des,obj.Kp{ind_subchain}(:,:,ind_task),rpyd,rpyd_des,obj.Kd{ind_subchain}(:,:,ind_task),rpydd_des);  

            % J_dot is just multiplied by qd
            b = b - J_dot;    
            
        end
        
    end    
         
 end
            
            
end            