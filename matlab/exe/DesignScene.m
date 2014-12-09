clear all
close all
clc

% in this variable we have to specify the name of the scenario:
% bot_scenario# where # is incremental
name_scenario = 'lbr_scenario4';
% with this variable i decide when i want to save the designed scenario
save_now = false;

subchain1 = [7 7 7];
target_link{1} = subchain1;
% reference parameters
type = {'cartesian_x','cartesian_x','cartesian_x'};
control_type = {'tracking','regulation','regulation'};
type_of_traj = {'func','func','func'};
traj = {'circular','none','none'};
time_law = {'linear','none','none'};

%parameters first chains
geom_parameters{1,1} = [0.2 0 -pi/2 -pi/4 0 -0.5 0.3]; % Circular trajectory 
geom_parameters{1,2} = [-0.2 -0.5 0.55]; % orientation regulation
geom_parameters{1,3} = [-0.2 -0.5 0.55]; % orientation regulation

time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

dim_of_task{1,1}={[1;1;1]}; dim_of_task{1,2}={[1;1;1]}; dim_of_task{1,3}={[1;1;1]};

%% robot
bot = MdlLBR4p();

%% reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();


%% plot scene

%%%;;

global G_OB;

hold on;axis equal;

p_tot=[];
for t=time_struct.ti:time_struct.step:time_struct.tf
 
	p_cur=reference.GetTraj(1,1,t);
	p_tot = [p_tot,p_cur];

end
    
plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));
repulsive_point = [-0.125538272258140 -0.5 0.455692460313405];
attractive_point1 = [-0.2 -0.5 0.55];
attractive_point2 = [-0.05 -0.5 0.35];
scatter3(repulsive_point(1,1),repulsive_point(1,2),repulsive_point(1,3), 130);
scatter3(attractive_point1(1,1),attractive_point1(1,2),attractive_point1(1,3), 130);
scatter3(attractive_point2(1,1),attractive_point2(1,2),attractive_point2(1,3), 130);
% global obstacle
ob1 = Obstacle(repulsive_point,'repellers',[]);
G_OB = [ob1];

%%%EOF

bot.plot(qz);
%bot.teach();

%% DO NOT CHANGE THIS PART!

if(save_now)
    % backup data 
    allpath=which('FindData.m');
    path=fileparts(allpath);
    rawTextFromStorage = fileread(which(mfilename));
    rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');
    fileID = fopen(strcat(path,'/scenario/',name_scenario,'.txt'),'w');
    fprintf(fileID,'%s',rawTextFromStorage);
    fclose(fileID);
    disp('DONE!')
end





