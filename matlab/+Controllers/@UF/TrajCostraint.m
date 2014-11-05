function [b,J] = TrajCostraint(obj,index,t,q,qd,x,xd,rpy,rpyd)

 if(strcmp(obj.reference.type(index),'joint'))
    
    else
        
        if(strcmp(obj.reference.type(index),'cartesian_x'))
        
            if(strcmp(obj.reference.control_type(index),'regulation'))
               
                [J,J_dot] = ReshapeJacobian(obj.GetNumLinks(),obj.subchains.sub_chains(index).jacob0(q(1:obj.GetNumSubLinks(index)),'trans'),obj.subchains.sub_chains(index).jacob_dot(q,qd),obj.references.mask(index));
                [x_des] = obj.reference.GetTraj(index,t);
                b = PD(x,x_des,obj.Kp(:,:,index),xd,zeros(1,3),obj.Kd(:,:,index),zeros(1,3));
                
                % J_dot is just multiplied by qd
                b = b - J_dot;
                
                return;
                
            elseif(strcmp(obj.reference.control_type(index),'tracking'))
                
                [J,J_dot] = ReshapeJacobian(obj.GetNumLinks(),obj.subchains.sub_chains(index).jacob0(q(1:obj.GetNumSubLinks(index)),'trans'),obj.subchains.sub_chains(index).jacob_dot(q,qd),obj.references.mask(index));
                [x_des,xd_des,xdd_des] = obj.reference.GetTraj(index,t);
                b = PD(x,x_des,obj.Kp(:,:,index),xd,xd_des,obj.Kd(:,:,index),xdd_des);  
                
                % J_dot is just multiplied by qd
                b = b - J_dot;
                
                return;
            end
            
        elseif(strcmp(obj.reference.type(index),'cartesian_rpy'))    
            
        end    
         
 end


            
            
end            