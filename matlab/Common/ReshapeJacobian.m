%#TODO add the functionalities for the mask
% here i make the hypothesis that i receive the jacobian of the subchain
% with real subchain i have to modify this function
function [J,J_dot,Fc] = ReshapeJacobian(J_old,J_dot_old,Fc_old,tot_link,sub_link,mask,trans_or_rot)

  % i have to insert the mask to select only a subset of the task jacobian
   % TODO update this part by removing the column that i do not need
   % directly in the jacobian given the sublink that i want to contol
%    if(sub_link<tot_link)
%         diff =tot_link - sub_link;
%         mask_mat = zeros(6,diff);
%         J_old(:,sub_link+1:end) = mask_mat;
%    end
  
   if(isempty(J_dot_old))
       if(strcmp(trans_or_rot,'trans'))  
          J = J_old(1:3,:);
          Fc = Fc_old(1:3,:);
       elseif(strcmp(trans_or_rot,'rot'))   
          J = J_old(4:6,:);
          Fc = Fc_old(4:6,:);
       else
           J=J_old;
           Fc = Fc_old;
       end
   else
       if(strcmp(trans_or_rot,'trans'))
          J_dot = J_dot_old(1:3,1);   
          J = J_old(1:3,:);
           Fc = Fc_old(1:3,:);
       elseif(strcmp(trans_or_rot,'rot'))   
          J_dot = J_dot_old(4:6,1);
          J = J_old(4:6,:);
          Fc = Fc_old(4:6,:);
       else
           J_dot = J_dot_old;
           J=J_old;
           Fc = Fc_old;
       end
   end
   
end