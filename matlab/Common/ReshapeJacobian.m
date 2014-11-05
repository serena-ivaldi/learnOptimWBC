%#TODO add the functionalities for the mask
% here i make the hypothesis that i receive the jacobian of the subchain
% with real subchain i have to modify this function
function [J,J_dot] = ReshapeJacobian(num_of_link,J_old,J_dot_old,mask,trans_or_rot)

   % here complete the jacobian for taking into account the dynamical
   % effect of the robot
   if(size(J_old,2)<num_of_link)
   J = [ J_old , zeros(size(J_old,1),num_of_link - size(J_old,2))];  
   end
   
   if(strcmp(trans_or_rot,'trans'))
      J_dot = J_dot_old(1:3,1);   
      J = J_old(1:3,:);
   elseif(strcmp(trans_or_rot,'rot'))   
      J_dot = J_dot_old(4:6,1);
      J = J_old(4:6,:);
   end
end