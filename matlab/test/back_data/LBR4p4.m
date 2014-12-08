%%%;;
%% comment
%this file describe 3 tasks a tracking task for the e-e on a circular
%trajectory and a two regulation task on the e-e to avoid an bostacle on
%the trajectory
%
%%


%SUBCHAIN PARAMETERS 
subchain1 = [7 7 7];
target_link{1} = subchain1;


%% Robot
[bot1] = MdlLBR4p();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
type = {'cartesian_x','cartesian_x','cartesian_x'};
control_type = {'tracking','regulation','regulation'};
type_of_traj = {'func','func','func'};
traj = {'circular','none','none'};
time_law = {'linear','none','none'};
%parameters first chains
geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3]; % Circular trajectory 
geom_parameters{1,2} = [-0.2 -0.5 0.55]; % orientation regulation
geom_parameters{1,3} = [-0.05 -0.5 0.35]; % orientation regulation

%CONTROLLER PARAMETERS
metric = {'M','M^(1/2)','M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
dim_of_task{1,1}={[1;1;1]};dim_of_task{1,2}={[1;1;1]};

kp = [700, 700, 700]; % row vector one for each chain
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
fitness= @fitness3;

%%%EOF