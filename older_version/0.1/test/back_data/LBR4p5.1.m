%%%;;


%% comment
%this file describe a regulation task for the e-e 
%trajectory and one reppeler task to avoid the obstacle on the elbow
%
%%


%SUBCHAIN PARAMETERS 
subchain1 = [7];
target_link{1} = subchain1;


%% Robot
[bot1] = MdlLBR4p();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
type = {'cartesian_x'};
control_type = {'regulation'};
type_of_traj = {'func'};
traj = {'none'};
time_law = {'none'};
%parameters first chains
geom_parameters{1,1} = [0.6 0 0]; 

% REPELLER PARAMETERS
% sceario dependant
rep_subchain = [3];
rep_target_link{1} = rep_subchain;
rep_type = {'cartesian_x'};
rep_mask {1,1}=[1,1,1];
rep_type_of_J_rep = {'DirectionCartesian'};
for ii=1:chains.GetNumChains()
    chain_dof(ii) = chains.GetNumLinks(ii);
end

%CONTROLLER PARAMETERS
% the metric change between regularized and not regularized because in the
% regularized case i have to do N^(-1) 
% not regularized case i have N^(-1/2)
metric = {'M'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
dim_of_task{1,1}={[1;1;1]};

kp = [700]; % row vector one for each chain
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


% INSTANCE PARAMETERS
fitness= @fitness6;


%%%EOF
