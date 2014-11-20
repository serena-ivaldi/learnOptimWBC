clear all
close all
clc

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');

%GENERAL PARAMETERS
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

% for fixed step simulation only
time_sym_struct = time_struct;
time_sym_struct.step = 0.01; 
fixed_step = false;

%SUBCHAIN PARAMETERS 
% we have to specify every value of the cell vector for consistency with
% the cycle inside the function 
target_link = [6 3];
% i consider only one perturbation for the whole robot chain
perturbation = 0;

% REFERENCE PARAMETERS
type = {'cartesian_x','cartesian_rpy'};
control_type = {'tracking','regulation'};
type_of_traj = {'func','func'};
traj = {'circular','none'};
time_law = {'linear','none'};
geom_parameters{1} = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3]; % Circular trajectory
geom_parameters{2} = [0 0 -pi/2]; % orientation regulation
%geom_parameters = [-0.2 0.3 0.2 0.2 0.3 0.2];% Rectilinear trajectory
%geom_parameters =  [-0.2 0.3 0.2]; % position regulation
time_parameters = [0.5]; % the way that im using time_parameters now is not usefull (i control the velocity of the trajectory through tf = final time)
dim_of_task{1}={[1;1;1]};
dim_of_task{2}={[1;1;1]};

%CONTROLLER PARAMETERS
metric = {'M^(1/2)';'M^(1/2)'};  % N^(1/2) = (M^(-1))^(1/2) = M^(1/2);        
ground_truth = false; 
%kp = 1500; %linear and exponential tracking
%kp = 1497  % regulation 
kp = [1497, 1500]; % row vector
K_p = zeros(3,3,size(kp,2));
K_d = zeros(3,3,size(kp,2));
for par = 1:size(kp,2)
    K_p(:,:,par) = kp(par)*eye(3);  
    kd = 2*sqrt(kp(par));
    K_d(:,:,par) = kd*eye(3); 
end
combine_rule = {'sum'}; 
display_opt.step = 0.001;
display_opt.trajtrack = true;




%% Robot
[p560] = MdlPuma560(target_link,perturbation);


% INSTANCE PARAMETERS
qinit = qz;
qdinit = zeros(size(qz));
fitness= @(t)t;
options= [];
simulator_type = {'rbt'};

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(p560,type,control_type,traj,geom_parameters,time_law,time_parameters,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();



%% Alpha
% test alpha function
number_of_basis = 4;
redundancy = 1;
theta = ones(number_of_basis,reference.GetNumTasks());
alphas = RBF.BuildCellArray(reference.GetNumTasks(),time_struct,number_of_basis,redundancy,theta);


%alpha.PlotBasisFunction();


%% Controller 

% for using package function we have to call the name of the package before
% the constructor
controller = Controllers.UF(p560,reference,alphas,metric,ground_truth,K_p,K_d,combine_rule,display_opt);

%% Instance
% starting value of parameters
start_action = ones(1,reference.GetNumTasks()*number_of_basis);
min_action   = 3;
max_action   = 3;
explorationRate =0.1;%[0, 1]
niter = 10;

inst = Instance(controller,simulator_type,qinit,qdinit,time_sym_struct,fixed_step,options,fitness);
inst.CMAES(start_action,min_action,max_action,niter,explorationRate)







