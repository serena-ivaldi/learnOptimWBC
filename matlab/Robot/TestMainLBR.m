clear all
close all
clc




% we have to specify every value of the cell vector for consistency with
% the cycle inside the function 
subchain1 = [7 4];
target_link{1} = subchain1;
% i consider only one perturbation for the whole robot chain
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


geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 -0.7 0.6]; % Circular trajectory
geom_parameters{1,2} = [0 0 -pi/2]; % orientation regulation
geom_parameters{1,3} = [0 0 -pi/2]; % orientation regulation
%geom_parameters = [-0.2 0.3 0.2 0.2 0.3 0.2];% Rectilinear trajectory
%geom_parameters =  [-0.2 0.3 0.2]; % position regulation

time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.01;

dim_of_task{1,1}={[1;1;1]};dim_of_task{1,2}={[1;1;1]};dim_of_task{1,3}={[1;1;1]};
%% test substructure
[LBR4p] = MdlLBR4p();
robots{1} = LBR4p;

chains = SubChains(target_link,robots);
%% test  func references
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();


tic
for i = 1:chains.GetNumChains()
   for j = 1:chains.GetNumTasks(i)

       for t=time_struct.ti:time_struct.step:time_struct.tf

         [p_cur,pd_cur,pdd_cur]=reference.GetTraj(i,j,t);

       end 

   end
end
toc


%% test sampled reference 

% s = Linear(time_struct.tf,time_parameters);
% type_of_traj = 'sampled';
% test_parameters = [0.2 0 -pi/2 2*pi -pi/4 0 -0.5 0.3]; 
% if(strcmp(traj,'circular')) 
%     [p,pd,pdd,time] = Circular(s,time_struct,geom_parameters,type_of_traj);
%     [p_test,pd_test,pdd_test] = CircularTest(time_struct,'sampled',test_parameters);
% end

%% plot Trajectories and Robot

if(strcmp(type_of_traj{1,1},'func')) 
  
    p_tot=[];
    pd_tot=[];
    pdd_tot=[];
    for t=time_struct.ti:time_struct.step:time_struct.tf
        
        
        p_cur=feval(reference.trajectories{1}.p,t);
        pd_cur=feval(reference.trajectories{1}.pd,t);
        pdd_cur=feval(reference.trajectories{1}.pdd,t);

        p_tot = [p_tot,p_cur];
        pd_tot = [pd_tot,pd_cur];
        pdd_tot = [pdd_tot,pdd_cur];
        
    end
    
elseif(strcmp(type_of_traj{1,1},'sampled'))
    p_tot = reference.trajectories{1}.p;
    pd_tot = reference.trajectories{1}.pd;
    pdd_tot = reference.trajectories{1}.pdd;
end


hold on;axis equal;
% LBR4p.plot(qz);
plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));
%% alpha function
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;
alphas = ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);
%% test controller 

% test on J J_dot and fkine
chains.GetNumSubLinks(1,1)
qz_cur = qz(1:chains.GetNumSubLinks(1,1));
%structure of jacobian
fkin = chains.sub_chains{1}.fkine(qz_cur);
J = chains.sub_chains{1}.jacob0(qz_cur);
% J_dot is multiplied by qd inside the function so i have only to get the
% right portion
J_dot = chains.sub_chains{1}.jacob_dot(qz_cur,0.5*ones(1,chains.GetNumSubLinks(1,1)));


% one shot only for generate jacob_dot
% JacDotGen(chains.sub_chains{1},'/home/modugno/Documents');

metric = {'M','M^(1/2)','M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        

kp = [700, 700, 4997]; % row vector one for each chain
for i= 1:chains.GetNumChains();
   K_p = zeros(3,3,size(kp,2));
   K_d = zeros(3,3,size(kp,2));
   for par = 1:size(kp,2)
       K_p(:,:,par) = kp(i,par)*eye(3);  
       kd = 2*sqrt(kp(i,par));
       K_d(:,:,par) = kd*eye(3); 
   end
   Kp{i} = K_p;
   Kd{i} = K_d;
end

combine_rule = {'sum'}; 
display_opt.step = 0.01;
display_opt.trajtrack = true;

% for using package function we have to call the name of the package before
% the constructor
controller = Controllers.UF(chains,reference,alphas,metric,Kp,Kd,combine_rule,display_opt);


% generate starting conditions for every chains
qi{1} = qz;
qdi{1} = zeros(1,controller.subchains.sub_chains{1}.n);


tic
%options= odeset('MaxStep',0.001);
fixed_step = false;
time_sym_struct = time_struct;
time_sym_struct.step = 0.01;
[t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step);%,options);
toc



%LBR4p.plot(q{1});


%% test Alpha

% % test alpha function
% number_of_basis = 4;
% redundancy = 1;
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
% 
% theta = ones(number_of_basis,reference.GetNumTasks());
% alpha_cell = RBF.BuildCellArray(reference.GetNumTasks(),time_struct,number_of_basis,redundancy,theta);
% for i=1:reference.GetNumTasks()
% figure   
% plot(alpha_cell{i}.sample.time,alpha_cell{i}.sample.normvalues);   
%     
% end






