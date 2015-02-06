% test GHC
clear variables 
close all 
clc


ARuntimeParameters


%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

%% plot scenario
text = LoadScenario(name_scenario);
eval(text);

%% constraints

constraints_list={'vellimit','vellimit','torquelimit','torquelimit'};
%cdata1 = [1;1];
cdata1 = [1;1000];
cdata2 = [0;1000];
cdata3 = [1;2000];
cdata4 = [0;2000];
constraints_data = [cdata1, cdata2, cdata3, cdata4];%, cdata5];
constraints = ContrPart.Constraints(constraints_list,constraints_data);

%% test of constraints and obstacle

% cp=[0,0.8,0.6];
% distanza = G_OB(1).Normal(cp)
% norm(distanza) 
%% chained alpha 
matrix1 = [0 1 0;0 0 0;1 1 0];  % 2>1>3
matrix2 = [0 0 0;1 0 0;1 1 0];  % 1>2>3
matrix3 = [0 0 0;1 0 1;1 0 0];  % 1>3>2
matrix4 = [0 0 1;1 0 1;0 0 0];  % 3>1>2
matrix_value(:,:,1) = matrix1;
matrix_value(:,:,2) = matrix2;
matrix_value(:,:,3) = matrix3;
matrix_value(:,:,4) = matrix4;
ti =[2 6 8];


% matrix1 = [0 1 0;0 0 0;1 1 0];  % 2>1>3
% matrix2 = [0 0 0;1 0 0;1 1 0];  % 1>2>3
% matrix3 = [0 1 0;0 0 0;1 1 0];  % 2>1>3
% matrix_value(:,:,1) = matrix1;
% matrix_value(:,:,2) = matrix2;
% matrix_value(:,:,3) = matrix3;
% ti =[5 8];
% with trnasition interval 1.5 i have a strange behaviour
transition_interval = 1.5;

alphas = Alpha.ChainedAlpha.BuildCellArray(chains.GetNumChains(),matrix_value,ti,transition_interval,time_struct);




%% test of alpha

time_sym = time_sym_struct.ti:time_sym_struct.step:time_sym_struct.tf;

tic
i = 1;
for t = time_sym
    for j=1:size(alphas{1}.all_value,2)
        app(i,j) = alphas{1}.GetValue(t,j);
    end
    i = i+1;
end
toc
for j=1:size(app,2)
figure
plot(app(:,j))
end





%% GHC
% % row vector one for each chain
kp = [1000 1000 1000]; 
kd = [2*sqrt(kp) 2*sqrt(kp) 2*sqrt(kp)];

%UPDATE UF USING THIS ELEMENTS
for i= 1:chains.GetNumChains()
  
   for par = 1:chains.GetNumTasks(i)
       K_p = kp(i,par)*eye(reference.GetDimTask(i,par));  
       K_d = kd(i,par)*eye(reference.GetDimTask(i,par)); 
       Kp{i,par} = K_p;
       Kd{i,par} = K_d;
       
   end
   
end
%---

epsilon = 0.002;
regularization = 0.01;
max_time = 2000;
delta_t = time_sym_struct.tf*time_struct.step;
controller = Controllers.GHC(chains,reference,alphas,constraints,Kp,Kd,regularization,epsilon,delta_t,max_time);

%% Simulation
tic
[t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step);%,options);
toc
%% Display
fps = 200;
video = false;

if(~video)
   bot1.plot(q{1},'fps',fps);
else
   %at the end of the video simulation after chosing a good camera pos and
   %zoom
   % to see camera position call "campos" on the shell 
   % to see zoom call "get(gca,'CameraViewAngle')" on the shell
   allpath = which('FindData.m');
   path = fileparts(allpath);
   path = strcat(path,'/video');
   camera_position = [-7.5371   -1.1569   21.1612];
   zoom =  2.4702;
   set(gca,'CameraViewAngle',zoom);
   campos(camera_position)
   bot1.plot(q{1},'movie',path);
end

