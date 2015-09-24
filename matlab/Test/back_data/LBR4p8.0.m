%%%;;


%% comment
%this file describe a regulation task for the e-e another regulation task
%for elbow and a rest pose for all the joints
%
%
%% TYPE OF CONTROLLER 
CONTROLLERTYPE ='GHC';   % GHC or UF
%%

%SUBCHAIN PARAMETERS 
subchain1 = [7 3 7];
target_link{1} = subchain1;


%% Robot
[bot1] = MdlLBR4p();
robots{1} = bot1;
chains = SubChains(target_link,robots);
%%

% REFERENCE PARAMETERS
% REFERENCE PARAMETERS
traj_type = {'cartesian_x','cartesian_x','joint'};
control_type = {'regulation','regulation','regulation'};
type_of_traj = {'func','func','func'};
geometric_path = {'none','none','none'};
time_law = {'none','none','none'};
%parameters first chains
geom_parameters{1,1} = [0.813 0.006 0.6]; 
geom_parameters{1,2} = [-0.13 0.02 0.72];
geom_parameters{1,3} = [0 pi/2 0 -pi/2 0 pi/2 0];
dim_of_task{1,1}=[1;1;1] ;dim_of_task{1,2}= [1;1;1] ;dim_of_task{1,3}=ones(bot1.n,1);
%% FROM THIS POINT YOU CAN FIND PARAMETERS IN THE STATIC PARAMETERS FILE RELATED TO EACH ALGORITHM 

%%%EOF%%%;;

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


%% CONTROLLER PARAMETERS
% % row vector one for each chain
kp = [1200 1200 1200]; 
kd = [2*sqrt(kp) 2*sqrt(kp) 2*sqrt(kp)];

for i= 1:chains.GetNumChains()
  
   for par = 1:chains.GetNumTasks(i)
       K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
       K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
       Kp{i,par} = K_p;
       Kd{i,par} = K_d;
       
   end
   
end
%---


%%%EOF