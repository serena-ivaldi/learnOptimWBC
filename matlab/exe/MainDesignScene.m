clear all
close all
clc


% we have to specify every value of the cell vector for consistency with
% the cycle inside the function 
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

hold on;axis equal;

if(strcmp(type_of_traj{1,1},'func')) 
  
    p_tot=[];
    pd_tot=[];
    pdd_tot=[];
    for t=time_struct.ti:time_struct.step:time_struct.tf
        
        
        p_cur=feval(reference.trajectories{1,1}.p,t);
        pd_cur=feval(reference.trajectories{1,1}.pd,t);
        pdd_cur=feval(reference.trajectories{1,1}.pdd,t);

        p_tot = [p_tot,p_cur];
        pd_tot = [pd_tot,pd_cur];
        pdd_tot = [pdd_tot,pdd_cur];
        
    end
    
elseif(strcmp(type_of_traj{1,1},'sampled'))
    p_tot = reference.trajectories{1,1}.p;
    pd_tot = reference.trajectories{1,1}.pd;
    pdd_tot = reference.trajectories{1,1}.pdd;
end


plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));
repulsive_point = [-0.125538272258140 -0.5 0.455692460313405];
attractive_point1 = [-0.2 -0.5 0.55];
attractive_point2 = [-0.05 -0.5 0.35];
scatter3(repulsive_point(1,1),repulsive_point(1,2),repulsive_point(1,3), 130);
scatter3(attractive_point1(1,1),attractive_point1(1,2),attractive_point1(1,3), 130);
scatter3(attractive_point2(1,1),attractive_point2(1,2),attractive_point2(1,3), 130);
bot.plot(qz);
%bot.teach();









