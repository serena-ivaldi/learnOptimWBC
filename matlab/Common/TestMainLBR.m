clear all
close all
clc


% we have to specify every value of the cell vector for consistency with
% the cycle inside the function 
target_link = [7 7];
% i consider only one perturbation for the whole robot chain
perturbation = 0;
type = {'cartesian_x','cartesian_rpy','cartesian_rpy'};
control_type = {'tracking','regulation','regulation'};
type_of_traj = {'func','func','func'};
traj = {'circular','none','none'};
time_law = {'linear','none','none'};

% type = {'cartesian_rpy'};
% control_type = {'regulation'};
% type_of_traj = {'func'};
% traj = {'none'};
% time_law = {'none'};


geom_parameters{1} = [0.2 0 -pi/2 -pi/4 0 -0.6 0.5]; % Circular trajectory
geom_parameters{2} = [0 0 -pi/2]; % orientation regulation
geom_parameters{3} = [0 0 -pi/2]; % orientation regulation
%geom_parameters = [-0.2 0.3 0.2 0.2 0.3 0.2];% Rectilinear trajectory
%geom_parameters =  [-0.2 0.3 0.2]; % position regulation

time_parameters = [0.5]; % the way that im using time_parameters now is not usefull (i control the velocity of the trajectory through tf = final time)
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.001;

dim_of_task{1}={[1;1;1]};
dim_of_task{2}={[1;1;1]};
dim_of_task{3}={[1;1;1]};
%% test substructure
[LBR4p] = MDL_LBR4p(target_link,perturbation);

%% test  func references
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(LBR4p,type,control_type,traj,geom_parameters,time_law,time_parameters,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();


% perfomance measure 
% tic
% for index=1:reference.GetNumTasks()
%     
%     for t=time_struct.ti:time_struct.step:time_struct.tf
%         
%       [p_cur,pd_cur,pdd_cur]=reference.GetTraj(index,t);
%       
%     end 
%       
% end
% toc


%% test sampled reference 

% s = Linear(time_struct.tf,time_parameters);
% type_of_traj = 'sampled';
% test_parameters = [0.2 0 -pi/2 2*pi -pi/4 0 -0.5 0.3]; 
% if(strcmp(traj,'circular')) 
%     [p,pd,pdd,time] = Circular(s,time_struct,geom_parameters,type_of_traj);
%     [p_test,pd_test,pdd_test] = CircularTest(time_struct,'sampled',test_parameters);
% end

%% plot Trajectories and Robot

% if(strcmp(type_of_traj{1},'func')) 
%   
%     p_tot=[];
%     pd_tot=[];
%     pdd_tot=[];
%     for t=time_struct.ti:time_struct.step:time_struct.tf
%         
%         
%         p_cur=feval(reference.trajectories{1}.p,t);
%         pd_cur=feval(reference.trajectories{1}.pd,t);
%         pdd_cur=feval(reference.trajectories{1}.pdd,t);
% 
%         p_tot = [p_tot,p_cur];
%         pd_tot = [pd_tot,pd_cur];
%         pdd_tot = [pdd_tot,pdd_cur];
%         
%     end
%     
% elseif(strcmp(type_of_traj{1},'sampled'))
%     p_tot = reference.trajectories{1}.p;
%     pd_tot = reference.trajectories{1}.pd;
%     pdd_tot = reference.trajectories{1}.pdd;
% end
% 
% 
% hold on;axis equal;
% LBR4p.plot(qz);
% plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));
%% alpha function
parameters = zeros(1,10);
values=[1,1,1];
alphas = ConstantAlpha.BuildCellArray(values,time_struct);

%% test controller 

% test on J J_dot and fkine
LBR4p.GetNumSubLinks(1)
qz(1:LBR4p.GetNumSubLinks(1))
%structure of jacobian
fkin = LBR4p.sub_chains(1).fkine(qz(1:LBR4p.GetNumSubLinks(1)));
J = LBR4p.sub_chains(1).jacob0(qz(1:LBR4p.GetNumSubLinks(1))','trans');
% J_dot is multiplied by qd inside the function so i have only to get the
% right portion
J_dot = LBR4p.sub_chains(1).jacob_dot(qz(1:LBR4p.GetNumSubLinks(1)),0.5*ones(LBR4p.GetNumSubLinks(1),1));



metric = {'M';'M^(1/2)';'M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
ground_truth = false; 
%kp = 1500; %linear and exponential tracking
%kp = 1497  % regulation 
kp = [700, 700, 4997]; % row vector
K_p = zeros(3,3,size(kp,2));
K_d = zeros(3,3,size(kp,2));
for par = 1:size(kp,2)
    K_p(:,:,par) = kp(par)*eye(3);  
    kd = 2*sqrt(kp(par));
    K_d(:,:,par) = kd*eye(3); 
end
combine_rule = {'sum'}; 
display_opt.step = 0.01;
display_opt.trajtrack = true;

% for using package function we have to call the name of the package before
% the constructor
controller = Controllers.UF(LBR4p,reference,alphas,metric,ground_truth,K_p,K_d,combine_rule,display_opt);


tic
fixed_step = false;
time_sym_struct = time_struct;
time_sym_struct.step = 0.001;
options= odeset('MaxStep',0.001);
%[t, q, qd] = controller.subchains.nofriction().fdyn(time_sym_struct,controller,qz,zeros(1,controller.subchains.n),fixed_step);%,options);
toc

%controller.plot(q,t);


%% test Alpha

% % test alpha function
number_of_basis = 4;
redundancy = 1;
% alpha = RBF(time_struct,number_of_basis,redundancy);
% % the parameters have to be a column vector !!!!
% alpha.ComputeNumValue(ones(number_of_basis,1));
% %plot(alpha.sample.time,alpha.sample.normvalues);
% %alpha.PlotBasisFunction();
% 
% 
% %[zp ,zpd ,zpdd , scaled_time] = RecordTrajectory(5,0.01);
% 
% number_of_basis = 10;
% redundancy = 3;
% kp = 10;
% kd = 2*sqrt(kp);
% alpha_z = 0.1;
% number_of_pivot = 5;
% step = 0.01;
% alpha1 = DMP(time_struct,number_of_basis,redundancy,kp,kd,alpha_z);
% [p_init,v_init,p_end,v_end,theta] = alpha1.TrainByDraw(number_of_pivot,step);
% alpha1.ComputeNumValue(p_init,v_init,p_end,v_end,theta);
% % plot results
% figure
% plot(alpha1.sample.time,alpha1.sample.normvalues);
% figure
% alpha1.PlotBasisFunction();

theta = ones(number_of_basis,reference.GetNumTasks());
alpha_cell = RBF.BuildCellArray(reference.GetNumTasks(),time_struct,number_of_basis,redundancy,theta);
for i=1:reference.GetNumTasks()
figure   
plot(alpha_cell{i}.sample.time,alpha_cell{i}.sample.normvalues);   
    
end






