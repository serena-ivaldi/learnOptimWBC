params.config.ADD_NOISE_FT_SENSORS = round(rand(1)); %generate gaussian noise on input F/T sensor signals
params.config.FOOT_LIFT_FRONT = round(rand(1)); %0 is lifting the foot towards the back; 1 is lifting the foot towards the front
params.config.COM_DELTA = (randi(3)-1)/100; %when config.COM_DELTA = 0.02, move the CoM 0.02 m to the front (except during two feet balancing)


params.external_force.n_wrench_applications = randi(params.external_force.maximum_number_of_wrenches);
%at random times in the simulation
params.external_force.time_of_application = sort(round(params.tEnd * rand(params.external_force.n_wrench_applications, 1), 2));
%for random durations
params.external_force.duration = round(params.external_force.max_duration_of_wrenches * rand(params.external_force.n_wrench_applications, 1), 2);
%with a random magnitude, towards a random direction
params.external_force.magnitude  = params.external_force.max_force  * rand(params.external_force.n_wrench_applications, 1);
params.external_torque.magnitude = params.external_force.max_torque * rand(params.external_force.n_wrench_applications, 1);
params.external_force.wrench     = zeros(params.external_force.n_wrench_applications, 6);

gamma_force  = 2*pi*rand(params.external_force.n_wrench_applications,1);
theta_torque = 2*pi*rand(params.external_force.n_wrench_applications,1);
phi_torque   = 2*pi*rand(params.external_force.n_wrench_applications,1);
for i = 1 : params.external_force.n_wrench_applications
   params.external_force.wrench(i,1) = params.external_force.magnitude(i)*cos(gamma_force(i));
   params.external_force.wrench(i,2) = params.external_force.magnitude(i)*sin(gamma_force(i));
   params.external_force.wrench(i,4) = params.external_force.wrench(i)*sin(theta_torque(i))*cos(phi_torque(i));
   params.external_force.wrench(i,5) = params.external_force.wrench(i)*sin(theta_torque(i))*sin(phi_torque(i));
   params.external_force.wrench(i,6) = params.external_force.wrench(i)*cos(theta_torque(i));
end
params.external_force.wrench = round(params.external_force.wrench,2);
