clear all
close all
clc



target_link = [6];
perturbation = 0;
type = 'cartesian_x';
control_type = 'tracking';
traj = 'circular';
parameters = [1 1 1 1 1]; 
dim_of_task{1}={[1;1;1]};
%% test substructure
[p560] = MdlPuma560(target_link,perturbation);

%% test references
reference = References(p560,type,control_type,traj,parameters,dim_of_task);
reference.BuildTrajs()


T = 10;
step = 0.1;


p=[];
pd=[];
pdd=[];

% perfomance measure 
tic
for index=1:reference.GetNumTasks()
    
    for t=0:step:T
        
      [p_cur,pd_cur,pdd_cur]=reference.GetTraj(index,t);
      
    end 
      
end
toc

% plot function
for index=1:reference.GetNumTasks()
    
    for t=0:step:T
        
      [p_cur,pd_cur,pdd_cur]=reference.GetTraj(index,t);
      p = [p;p_cur];
      pd = [pd;pd_cur];
      pdd = [pdd;pdd_cur];
      
    end 
    
    Results{index} = [p pd pdd];
    p=[];
    pd=[];
    pdd=[];
      
end
plot3(Results{1}(:,1),Results{1}(:,2),Results{1}(:,3));


%% test controller 

% test on J J_dot and fkine
p560.GetNumSubLinks(1)
qz(1:p560.GetNumSubLinks(1))
%structure of jacobian
fkin = p560.sub_chains(1).fkine(qr(1:p560.GetNumSubLinks(1)));
J = p560.sub_chains(1).jacob0(qr(1:p560.GetNumSubLinks(1))','trans');
% J_dot is multiplied by qd inside the function so i have only to get the
% right portion
J_dot = p560.sub_chains(1).jacob_dot(qr(1:p560.GetNumSubLinks(1)),0.5*ones(p560.GetNumSubLinks(1)));

metric = 'M_inv';           
ground_truth = false; 
kp = 3;
K_p = kp*eye(3);  
kd = 1;
K_d = kd*eye(3);                
combine_rule = 'sum'; 







%% test instance