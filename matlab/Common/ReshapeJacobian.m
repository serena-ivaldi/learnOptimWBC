%#TODO add the functionalities for the mask
function [J,J_dot] = ReshapeJacobian(num_of_link,J_old,J_dot_old,mask,trans_or_rot)

   % here complete the jacobian for taking into account the dynamical
   % effect of the robot
   if(size(J,2)<num_of_link)
   J = [ J_old , zeros(size(J_old,1),num_of_link - size(J,2))];  
   end
   
   if(strcmp(trans_or_rot,'trans'))
      J_dot = J_dot_old(1:3,1);   
   elseif(strcmp(trans_or_rot,'rot'))   
      J_dot = J_dot_old(3:6,1);
   end
end