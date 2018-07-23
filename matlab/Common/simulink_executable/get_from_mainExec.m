%get results from running with mainExec

controller      = inst.input_4_run{4};
% pose_CoM_des
% pose_lFoot_des
% pose_rFoot_des
task_errors     = controller.simulation_results.task_errors;     %[nsamples x 12] matrix, [CoMx, CoMy, CoMz, OriRot, lFootx,lFooty,lFootz,lFootRot,rFootx, rFooty, rFootz, rFootRot]
joint_error     = controller.simulation_results.joint_error;     %[nsamples x nDOF]
feet_in_contact = controller.simulation_results.feet_in_contact; %[nsamples x 2]
pose_CoM        = controller.simulation_results.pose_CoM;        %[nsamples x 3]
pose_lFoot      = controller.simulation_results.pose_lFoot;      %[nsamples x 3]
pose_rFoot      = controller.simulation_results.pose_rFoot;      %[nsamples x 3]
time            = controller.simulation_results.time;            %[nsamples x 1]
zmpErr          = controller.simulation_results.zmpErr;          %[nsamples x 1]
support_polygon = controller.simulation_results.support_polygon; %[2 x 2 x nsamples]
ZMP             = controller.simulation_results.zmp;             %[nsamples x 3]
torques         = controller.simulation_results.torques;         %[nsamples x nDOF]
