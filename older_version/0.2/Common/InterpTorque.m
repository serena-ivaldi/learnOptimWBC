% i obtain the interpolated torque and in this way i can compute mean and
% variance
function torque=InterpTorque(controller,time_struct,interp_step)
   time = time_struct.ti:interp_step:time_struct.tf;
   if(size(controller.torques_time,2) == size(controller.torques,2))
      [unique_time,ia,ic] = unique(controller.torques_time);
   else
      diff = size(controller.torques_time,2) - size(controller.torques,2) + 1;
      controller.torques_time{1} = controller.torques_time(diff:end);
      [unique_time,ia,ic] = unique(controller.torques_time);
   end
   torque = [];
   for i = 1:size(controller.torques,1)
      unique_torque = controller.torques(i,ia);
      interp_torque = interp1(unique_time,unique_torque,time,'nearest');
      torque = [torque , interp_torque'];
   end 
end