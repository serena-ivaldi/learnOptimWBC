function [b,J] = TrajCostraint(obj,index,t,J_old,Jd_old,x,xd,rpy,rpyd)

 if(strcmp(obj.references.type(index),'joint'))
 
 else
        
    if(strcmp(obj.references.type(index),'cartesian_x'))
    
        if(strcmp(obj.references.control_type(index),'regulation'))

            [J,J_dot] = ReshapeJacobian(obj.subchains.GetNumLinks(),J_old,Jd_old,obj.references.mask(index),'trans');
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(index,t);
            b = PD(x,x_des,obj.Kp(:,:,index),xd,xd_des,obj.Kd(:,:,index),xdd_des);

            % J_dot is just multiplied by qd
            b = b - J_dot;

            return;

        elseif(strcmp(obj.references.control_type(index),'tracking'))
           
            [J,J_dot] = ReshapeJacobian(obj.subchains.GetNumLinks(),J_old,Jd_old,obj.references.mask(index),'trans');
            [x_des,xd_des,xdd_des] = obj.references.GetTraj(index,t);
            b = PD(x,x_des,obj.Kp(:,:,index),xd,xd_des,obj.Kd(:,:,index),xdd_des);  

            % J_dot is just multiplied by qd
            b = b - J_dot;

            return;
        end

    elseif(strcmp(obj.references.type(index),'cartesian_rpy'))    

    end    
         
 end
            
            
end            