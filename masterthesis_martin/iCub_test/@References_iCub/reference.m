subchain1 = [7];
target_link{1} = subchain1;
% REFERENCE PARAMETERS
traj_type = {'cartesian_x'};
control_type = {'tracking'};
type_of_traj = {'func'};
geometric_path = {'circular'};
time_law = {'linear'};
%parameters first chains
geom_parameters{1,1} = [0.3 pi/2 -pi/2 0 0 -0.5 0.5];
dim_of_task{1,1}=[1;1;1]; 
reference = References(target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

