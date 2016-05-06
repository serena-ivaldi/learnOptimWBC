%%%;;

%% comment
%this file describe a test task for the elastic reference
%
%
%% TYPE OF CONTROLLER 
CONTROLLERTYPE ='UF';   % GHC or UF
%%

%SUBCHAIN PARAMETERS 
subchain1 = [7];
target_link{1} = subchain1;

%% Robot
[bot1] =  MdlLBR4pSimple();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%  REFERENCE PARAMETERS
deg = pi/180;
% primary trajectory
% traj_type = {'impedance'};
% control_type = {'x'};
% type_of_traj = {'func'};
% geometric_path = {'fixed'};
% time_law = {'none'};
% %parameters first chains
% geom_parameters{1,1} = [0.30 -0.71 0.5]; 
% %geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
% dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.n,1); %dim_of_task{1,4}=ones(bot1.n,1);

traj_type = {'cartesian'};
control_type = {'x'};
type_of_traj = {'func'};
geometric_path = {'circular'};
time_law = {'linear'};
%parameters first chains
geom_parameters{1,1} = [0.2 pi/2 pi/2 0 0.0 -0.75 0.5]; % regulation
dim_of_task{1,1}={[1;1;1]};


% traj_type = {'cartesian'};
% control_type = {'rpy'};
% type_of_traj = {'func'};
% geometric_path = {'fixed'};
% time_law = {'none'};
% %parameters first chains
% geom_parameters{1,1} = [pi/2 0 -pi/2]; 
% %geom_parameters{1,2} = [-0.309 -0.469 0.581]; geom_parameters{1,3} = [120 116 90 0 0 0]* deg; geom_parameters{1,4} = [0 0 0 0 0 0 0];
% dim_of_task{1,1}=[1;1;1]; %dim_of_task{1,2}= [1;1;1]; dim_of_task{1,3}= ones(bot1.n,1); %dim_of_task{1,4}=ones(bot1.n,1);

% secondary trajectory
traj_type_sec = {'none'};
control_type_sec = {'rpy'};
type_of_traj_sec = {'func'};
geometric_path_sec = {'fixed'};
time_law_sec = {'linear'};
%parameters first chains
geom_parameters_sec{1,1} = [pi/2 0 -pi/2]; % regulation
dim_of_task_sec{1,1}={[1;1;1]};

% traj_type_sec = {'cartesian_x'};
% control_type_sec = {'regulation'};
% type_of_traj_sec = {'none'};
% geometric_path_sec = {'none'};
% time_law_sec = {'linear'};
% parameters first chains
% geom_parameters_sec{1,1} = [0.30 -0.71 0.5]; % regulation
% dim_of_task_sec{1,1}={[1;1;1]};


%% FROM THIS POINT YOU CAN FIND PARAMETERS IN THE STATIC PARAMETERS FILE RELATED TO EACH ALGORITHM 

%%%EOF%%%;;

%MOVED INTO  static parameters

% REPELLER PARAMETERS 
% scenario dependant
% rep_subchain = [7];
% rep_target_link{1} = rep_subchain;
% rep_type = {'cartesian_x'};
% rep_mask {1,1}=[1,1,1]; 
% rep_type_of_J_rep = {'DirectionCartesian'};
% for ii=1:chains.GetNumChains()
%     chain_dof(ii) = chains.GetNumLinks(ii);
% end


%MOVED INTO  runtime parameters

% %CONTROLLER PARAMETERS
% % the metric change between regularized and not regularized because in the
% % regularized case i have to do N^(-1) 
% % not regularized case i have N^(-1/2)
% metric = {'M','M','M','M'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        
% 
% 
% kp = [50,10,5,700]; % row vector one for each chain 50,1,5
% kd = [2*sqrt(kp),2*sqrt(kp),2*sqrt(kp),2*sqrt(kp)];
% 
% for i= 1:chains.GetNumChains()
%    for par = 1:chains.GetNumTasks(i)
%        K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
%        K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
%        Kp{i,par} = K_p;
%        Kd{i,par} = K_d;
%    end
%    
% end



%%%EOF