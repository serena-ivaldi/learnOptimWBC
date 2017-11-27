% i obtain the interpolated torque and in this way i can compute mean and
% variance
function torque = InterpTorque(controller,time_struct,interp_step)
   time = time_struct.ti:interp_step:time_struct.tf;
   if(size(controller.torques_time,2) == size(controller.torques,2))
      [unique_time,ia,~] = unique(controller.torques_time);
   else
      diff = size(controller.torques_time,2) - size(controller.torques,2) + 1;
      controller.torques_time{1} = controller.torques_time(diff:end);
      [unique_time,ia,~] = unique(controller.torques_time);
   end
   unique_torque = controller.torques(1,ia);
   interp_torque = interp1(unique_time,unique_torque,time,'nearest');

   vlen   = size(interp_torque,2);
   ndof   = size(controller.torques,1);
   torque = zeros(vlen,ndof);

   torque(1:vlen,1) = interp_torque.';
   for i = 2:ndof
      unique_torque = controller.torques(i,ia);
      interp_torque = interp1(unique_time,unique_torque,time,'nearest');
      torque(1:vlen,i) = interp_torque.';
   end
   % torque = [];
   % for i = 1:size(controller.torques,1)
   %     unique_torque = controller.torques(i,ia);
   %     interp_torque = interp1(unique_time,unique_torque,time,'nearest');
   %     torque = [torque , interp_torque'];
   % end
end
