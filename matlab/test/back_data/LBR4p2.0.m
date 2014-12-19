%%%;;

%% comment
%this file describe 2 regulation tasks a  to avoid a wall that act as an
%obstacle
%
%%


%SUBCHAIN PARAMETERS 
subchain1 = [7 3];
target_link{1} = subchain1;


%% Robot
[bot1] = MdlLBR4p();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
type = {'cartesian_x','cartesian_x'};
control_type = {'regulation','regulation'};
type_of_traj = {'func','func'};
traj = {'none','none'};
time_law = {'none','none'};
%parameters first chains
geom_parameters{1,1} = [0, -0.7,0.5100]; % position 
geom_parameters{1,2} = [-0.2 -0.4000 0.3100]; % position


% REPELLER PARAMETERS (EMPTY)
% scenario dependant
rep_subchain = [];
rep_target_link{1} = rep_subchain;
rep_type = {' '};
rep_mask {1,1}=[1,1,1];
rep_type_of_J_rep = {'DirectionCartesian'};
for ii=1:chains.GetNumChains()
    chain_dof(ii) = chains.GetNumLinks(ii);
end


%CONTROLLER PARAMETERS
metric = {'M','M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
dim_of_task{1,1}={[1;1;1]};dim_of_task{1,2}={[1;1;1]};

kp = [700, 700]; % row vector one for each chain
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
fitness= @fitness4;
%%%EOF