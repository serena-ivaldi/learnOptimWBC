%#TODO add the functionalities for the mask
% here i make the hypothesis that i receive the jacobian of the subchain
% with real subchain i have to modify this function
function [J,J_dot] = ReshapeJacobian(J_old,J_dot_old,tot_link,sub_link,mask,trans_or_rot)

  % i have to insert the mask to select only a subset of the task jacobian
   
   if(sub_link<tot_link)
        diff =tot_link - sub_link;
        mask_mat = zeros(6,diff);
        J_old(:,sub_link+1:end) = mask_mat;
   end
   
   if(strcmp(trans_or_rot,'trans'))
      J_dot = J_dot_old(1:3,1);   
      J = J_old(1:3,:);
   elseif(strcmp(trans_or_rot,'rot'))   
      J_dot = J_dot_old(4:6,1);
      J = J_old(4:6,:);
   end
end