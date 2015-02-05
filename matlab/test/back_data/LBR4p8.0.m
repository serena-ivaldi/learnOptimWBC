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
type = {'cartesian_x','cartesian_x','joint'};
control_type = {'regulation','regulation','regulation'};
type_of_traj = {'func','func','func'};
traj = {'none','none','none'};
time_law = {'none','none','none'};
%parameters first chains
geom_parameters{1,1} = [0.813 0.006 0.6]; 
geom_parameters{1,2} = [-0.13 0.02 0.72];
geom_parameters{1,3} = [0 pi/2 0 -pi/2 0 pi/2 0];
dim_of_task{1,1}=[1;1;1] ;dim_of_task{1,2}= [1;1;1] ;dim_of_task{1,3}=ones(bot1.n,1);
%% parameter dependant on the type of controller 
switch CONTROLLERTYPE
    case 'UF'
        UF_StaticParameters
    case 'GHC'
        GHC_StaticParameters
    otherwise
        warning('Unexpected control method')
end
%%%EOF