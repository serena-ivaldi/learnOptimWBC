clear all
close all
clc


% we have to specify every value of the cell vector for consistency with
% the cycle inside the function 
target_link = [6 3];
% i consider only one perturbation for the whole robot chain
perturbation = 0;
type = {'cartesian_x','cartesian_rpy'};
control_type = {'tracking','regulation'};
type_of_traj = {'func','func'};
traj = {'circular','none'};
time_law = {'linear','none'};

% type = {'cartesian_rpy'};
% control_type = {'regulation'};
% type_of_traj = {'func'};
% traj = {'none'};
% time_law = {'none'};


geom_parameters{1} = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3]; % Circular trajectory
geom_parameters{2} = [0 0 -pi/2]; % orientation regulation
%geom_parameters = [-0.2 0.3 0.2 0.2 0.3 0.2];% Rectilinear trajectory
%geom_parameters =  [-0.2 0.3 0.2]; % position regulation

time_parameters = [0.5]; % the way that im using time_parameters now is not usefull (i control the velocity of the trajectory through tf = final time)
time_struct.ti = 0;
time_struct.tf = 20;
time_struct.step = 0.1;

dim_of_task{1}={[1;1;1]};
dim_of_task{2}={[1;1;1]};
%% test substructure
[p560] = MdlPuma560(target_link,perturbation);

%% test  func references
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(p560,type,control_type,traj,geom_parameters,time_law,time_parameters,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();


% perfomance measure 
tic
for index=1:reference.GetNumTasks()
    
    for t=time_struct.ti:time_struct.step:time_struct.tf
        
      [p_cur,pd_cur,pdd_cur]=reference.GetTraj(index,t);
      
    end 
      
end
toc


%% test sampled reference 

s = Linear(time_struct.tf,time_parameters);
type_of_traj = 'sampled';
test_parameters = [0.2 0 -pi/2 2*pi -pi/4 0 -0.5 0.3]; 
if(strcmp(traj,'circular')) 
    [p,pd,pdd,time] = Circular(s,time_struct,geom_parameters,type_of_traj);
    [p_test,pd_test,pdd_test] = CircularTest(time_struct,'sampled',test_parameters);
end

%% plot trajectories

% if(strcmp(type_of_traj,'func')) 
%   
%     p_tot=[];
%     pd_tot=[];
%     pdd_tot=[];
%     for t=time_struct.ti:time_struct.step:time_struct.tf
%         
%         normtime = NormalizeTime(t,time_struct.ti,time_struct.tf); 
%         p_cur=feval(p,normtime);
%         pd_cur=feval(pd,normtime);
%         pdd_cur=feval(pdd,normtime);
% 
%         p_tot = [p_tot,p_cur];
%         pd_tot = [pd_tot,pd_cur];
%         pdd_tot = [pdd_tot,pdd_cur];
%         
%     end
%     
% elseif(strcmp(type_of_traj,'sampled'))
%     p_tot = p;
%     pd_tot = pd;
%     pdd_tot = pdd;
% end
% 
% plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));
% hold on;
% plot3(p_test(1,1:end),p_test(2,1:end),p_test(3,1:end),'r');
% p560.plot3d(qz,'path','/home/modugno/Documents/toolbox/arte/arte3.2.3/robots/UNIMATE/puma560');
%p560.plot(qz);
%% test controller 

% test on J J_dot and fkine
p560.GetNumSubLinks(1)
qz(1:p560.GetNumSubLinks(1))
%structure of jacobian
fkin = p560.sub_chains(1).fkine(qr(1:p560.GetNumSubLinks(1)));
J = p560.sub_chains(1).jacob0(qr(1:p560.GetNumSubLinks(1))','trans');
% J_dot is multiplied by qd inside the function so i have only to get the
% right portion
J_dot = p560.sub_chains(1).jacob_dot(qz(1:p560.GetNumSubLinks(1)),0.5*ones(p560.GetNumSubLinks(1),1));



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

% for using package function we have to call the name of the package before
% the constructor
controller = Controllers.UF(p560,reference,metric,ground_truth,K_p,K_d,combine_rule,display_opt);


tic
options= odeset('MaxStep',0.001);
%[t, q, qd] = controller.subchains.nofriction().fdyn(time_struct.tf,controller,qz,zeros(1,controller.subchains.n),options);
toc


%controller.plot3d(q,t,'path','/home/modugno/Documents/toolbox/arte/arte3.2.3/robots/UNIMATE/puma560')
%controller.plot(q,t);


%% test instance

% test alpha function
number_of_basis = 4;
redundancy = 3;
alpha = RBF(time_struct,number_of_basis,redundancy);
% parameters have to be a column vector !!!!
alpha.ComputeNumValue(ones(number_of_basis,1));
plot(alpha.sample.time,alpha.sample.normvalues);
%alpha.PlotBasisFunction();









