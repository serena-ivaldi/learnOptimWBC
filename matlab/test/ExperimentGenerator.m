% EXPERIMENTAL SETTINGS GENERATOR


%GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

% for simulation 
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
fixed_step = false;

%SUBCHAIN PARAMETERS 
subchain1 = [7 3];
target_link{1} = subchain1;


%% Robot
[LBR4p] = MdlLBR4p();
robots{1} = LBR4p;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERs
type = {'cartesian_x','cartesian_x'};
control_type = {'regulation','regulation'};
type_of_traj = {'func','func'};
traj = {'none','none'};
time_law = {'none','none'};

geom_parameters{1,1} = [0, -0.7,0.5100]; %position regulation
geom_parameters{1,2} = [-0.2 -0.4000 0.3100];  %position regulation

dim_of_task{1,1}={[1;1;1]};dim_of_task{1,2}={[1;1;1]};dim_of_task{1,3}={[1;1;1]};

%CONTROLLER PARAMETERS
metric = {'M','M^(1/2)','M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        

kp = [700, 700, 4997]; % row vector one for each chain
for i= 1:chains.GetNumChains()
   K_p = zeros(3,3,size(kp,2));
   K_d = zeros(3,3,size(kp,2));
   for par = 1:chains.GetNumTasks(i)
       K_p(:,:,par) = kp(i,par)*eye(3);  
       kd = 2*sqrt(kp(i,par));
       K_d(:,:,par) = kd*eye(3); 
   end
   Kp{i} = K_p;
   Kd{i} = K_d;
end

max_time = 50;
combine_rule = {'sum'}; 
display_opt.step = 0.01;
display_opt.trajtrack = true;



save 