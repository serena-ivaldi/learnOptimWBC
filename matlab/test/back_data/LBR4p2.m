%%%;;
%% comment
%this file describe two position regulation tasks on lwr one on the e-e one on the elbow
% and this data setting work with scenario 2 (wall scenario)
%
%%


%SUBCHAIN PARAMETERS 
subchain1 = [7 3];
target_link{1} = subchain1;


%% Robot
[LBR4p] = MdlLBR4p();
robots{1} = LBR4p;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
type = {'cartesian_x','cartesian_x'};
control_type = {'regulation','regulation'};
type_of_traj = {'func','func'};
traj = {'none','none'};
time_law = {'none','none'};

geom_parameters{1,1} = [0, -0.7,0.5100]; %position regulation
geom_parameters{1,2} = [-0.2 -0.4000 0.3100];  %position regulation

dim_of_task{1,1}={[1;1;1]};dim_of_task{1,2}={[1;1;1]};

%CONTROLLER PARAMETERS
metric = {'M','M^(1/2)'};  % N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        

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
fitness= @fitness2;

%%%EOF