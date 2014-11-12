clear all
close all
clc



target_link = [6];
perturbation = 0;
type = {'cartesian_x'};
control_type = {'tracking'};
traj = {'circular'};
type_of_traj = {'func'};
%  x_centre = parameters(1);
%  y_centre = parameters(2);
%  z_centre = parameters(3);
%  rad      = parameters(4);
%  phi      = parameters(5);
%  theta    = parameters(6);
%  w        = parameters(7);
%  w0       = parameters(8);
parameters = [0 -0.5 0.3 0.3 0 -pi/2 1 -pi/4]; 
dim_of_task{1}={[1;1;1]};
%% test substructure
[p560] = MdlPuma560(target_link,perturbation);

%% test references
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(p560,type,control_type,traj,parameters,dim_of_task,type_of_traj);
reference.BuildTrajs();


Time = 10;
step = 0.1;


p=[];
pd=[];
pdd=[];

% perfomance measure 
tic
for index=1:reference.GetNumTasks()
    
    for t=0:step:Time
        
      [p_cur,pd_cur,pdd_cur]=reference.GetTraj(index,t);
      
    end 
      
end
toc

% plot function
% for index=1:reference.GetNumTasks()
%     
%     for t=0:step:Time
%         
%       [p_cur,pd_cur,pdd_cur]=reference.GetTraj(index,t);
%       p = [p;p_cur];
%       pd = [pd;pd_cur];
%       pdd = [pdd;pdd_cur];
%       
%     end 
%     
%     Results{index} = [p pd pdd];
%     p=[];
%     pd=[];
%     pdd=[];
%       
% end
% %plot starting point
% %plot3(Results{1}(1,1),Results{1}(1,2),Results{1}(1,3),'-.r*','MarkerSize',10);
% hold on
% %plot other points
% plot3(Results{1}(1:end,1),Results{1}(1:end,2),Results{1}(1:end,3));
% phi      = parameters(5);
% theta    = parameters(6);
% %plot vector u (green)
% quiver3(parameters(1),parameters(2),parameters(3),-sin(theta),cos(theta),0,'g');
% %plot vector nxv (yellow)
% quiver3(parameters(1),parameters(2),parameters(3),cos(theta)*cos(phi),cos(theta)*sin(phi),-sin(theta),'y');
%% test symbolic reference 
Ti = 0;
Tf = 10;
step = 0.1;
s = Exponential();
type = 'sampled';
[p,pd,pdd,time] = CircularSymbolic(s,Ti,Tf,step,parameters,type);


%% test controller 

% % test on J J_dot and fkine
% p560.GetNumSubLinks(1)
% qz(1:p560.GetNumSubLinks(1))
% %structure of jacobian
% fkin = p560.sub_chains(1).fkine(qr(1:p560.GetNumSubLinks(1)));
% J = p560.sub_chains(1).jacob0(qr(1:p560.GetNumSubLinks(1))','trans');
% % J_dot is multiplied by qd inside the function so i have only to get the
% % right portion
% J_dot = p560.sub_chains(1).jacob_dot(qz(1:p560.GetNumSubLinks(1)),0.5*ones(p560.GetNumSubLinks(1),1));
% 
% 
% 
% metric = {'M^(1/2)'};  % N^(1/2) = (M^(-1))^(1/2) = M^(1/2);        
% ground_truth = false; 
% kp = 100;
% K_p = kp*eye(3);  
% kd = 2*sqrt(kp);
% K_d = kd*eye(3);                
% combine_rule = {'sum'}; 
% display_opt.step = 0.001;
% display_opt.trajtrack = true;
% 
% controller = Controllers.UF(p560,reference,metric,ground_truth,K_p,K_d,combine_rule,display_opt);
% 
% 
% tic
% %options= odeset('MaxStep',0.1);
% [t ,q ,qd] = controller.subchains.nofriction().fdyn(Time,controller,qz(1,1:controller.subchains.GetNumSubLinks(1)),zeros(1,controller.subchains.GetNumSubLinks(1)));%,options);
% toc
% 
% 
% controller.plot3d(q,t,'path','/home/vale/Documents/MatlabToolbox/rvctools/contrib/arte/robots/UNIMATE/puma560')
%controller.plot(q,t);


%% test instance