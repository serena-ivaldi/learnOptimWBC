function [b,J] = TrajCostraint(obj,index,t,q,qd,x,xd,rpy,rpyd)

 if(strcmp(obj.reference.type(index),'joint'))
    
    else
        
        if(strcmp(obj.reference.type(index),'cartesian_x'))
        
            if(strcmp(obj.reference.control_type(index),'regulation'))
               
                %#TODO implement function for reshaping jacobian respect of number of degrees of freedom and mask
                J = ReshapeJacobian(J_old,mask);
                b = PD(x,x_des,obj.Kp(:,:,index),xd,zeros(1,3),obj.Kd(:,:,index),zeros(1,3));
                
                %obj.Jd_vec*q
                return;
                
            elseif(strcmp(obj.reference.control_type(index),'tracking'))
                
                b = PD(x,x_des,obj.Kp(:,:,index),xd,xd_des,obj.Kd(:,:,index),xdd_des);  
                %obj.Jd_vec*q
                return;
            end
            
        elseif(strcmp(obj.reference.type(index),'cartesian_rpy'))    
            
        end    
         
 end


            
            
end            