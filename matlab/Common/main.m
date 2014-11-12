clear all
close all
clc



target_link = [6];
perturbation = 0;
type = {'cartesian_x'};
control_type = {'tracking'};
type_of_traj = {'func'};
traj = {'circular'};
time_law = {'exponential'};
% rad_val      = geom_parameters(1);
% phi_val      = geom_parameters(2);
% theta_val    = geom_parameters(3);
% wzero_val    = geom_parameters(4);
% x_centre_val = geom_parameters(5);
% y_centre_val = geom_parameters(6);
% z_centre_val = geom_parameters(7);
geom_parameters = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3];   
time_parameters = [0.5]; 
time_struct.ti = 0;
time_struct.tf = 20;
time_struct.step = 0.1;

dim_of_task{1}={[1;1;1]};
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
[p,pd,pdd,time] = Circular(s,time_struct,geom_parameters,type_of_traj);
[p_test,pd_test,pdd_test] = CircularTest(time_struct,'sampled',test_parameters);

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



metric = {'M^(1/2)'};  % N^(1/2) = (M^(-1))^(1/2) = M^(1/2);        
ground_truth = false; 
kp = 1500; %linear and exponential
K_p = kp*eye(3);  
kd = 2*sqrt(kp);
K_d = kd*eye(3);                
combine_rule = {'sum'}; 
display_opt.step = 0.001;
display_opt.trajtrack = true;

controller = Controllers.UF(p560,reference,metric,ground_truth,K_p,K_d,combine_rule,display_opt);


tic
%options= odeset('MaxStep',0.1);
[t, q, qd] = controller.subchains.nofriction().fdyn(time_struct.tf,controller,qz(1,1:controller.subchains.GetNumSubLinks(1)),zeros(1,controller.subchains.GetNumSubLinks(1)));%,options);
toc


controller.plot3d(q,t,'path','/home/modugno/Documents/toolbox/arte/arte3.2.3/robots/UNIMATE/puma560')
%controller.plot(q,t);


%% test instance