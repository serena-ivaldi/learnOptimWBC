clear all
close all
clc

% in this variable we have to specify the name of the scenario:
% bot_scenario# where # is incremental
name_scenario = 'lbr_scenario9';
% with this variable i decide when i want to save the designed scenario
save_now = true;

plot_subchain1 = [7];
plot_target_link{1} = plot_subchain1;
% reference parameters
plot_type = {'cartesian_x'};
plot_control_type = {'regulation'};
plot_type_of_traj = {'func'};
plot_traj = {'none'};
plot_time_law = {'none'};

%parameters first chains
plot_geom_parameters{1,1} = [0.643 0 -0.174];
plot_time_struct.ti = 0;
plot_time_struct.tf = 10;
plot_time_struct.step = 0.1;

plot_dim_of_task{1,1}={[1;1;1]};

%% robot
plot_bot = MdlLBR4p();

%% reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
plot_reference = References(plot_target_link,plot_type,plot_control_type,plot_traj,plot_geom_parameters,plot_time_law,plot_time_struct,plot_dim_of_task,plot_type_of_traj);
plot_reference.BuildTrajs();


%% plot scene

%%%;;

global G_OB;

hold on;axis equal;

step = 0.01;
[X Y Z]=meshgrid(0.75, -0.1 : step : 0.3, 0.4: step : 0.9);
%Y = -0.4*ones(1,size(X,1));
for i=1:size(X,3) 
    scatter3(X(:,:,i),Y(:,:,i),Z(:,:,i))
end

attractive_point1 = [0.813 0.006 0.6]; % for end effector
attractive_point2 = [-0.13 0.02 0.72];
scatter3(attractive_point1(1,1),attractive_point1(1,2),attractive_point1(1,3), 130);
scatter3(attractive_point2(1,1),attractive_point2(1,2),attractive_point2(1,3), 130);


rapresentation.X = X;
rapresentation.Y = Y;
rapresentation.Z = Z;
ob1 = Obstacle(rapresentation,'wall',0.002);
G_OB = [ob1];



%%%EOF


%plot_bot.plot(qz);
plot_bot.teach();

%% DO NOT CHANGE THIS PART!

if(save_now)
    % backup data 
    allpath=which('FindData.m');
    path=fileparts(allpath);
    rawTextFromStorage = fileread(which(mfilename));
    rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match','once');    
    existence = exist(strcat(path,'/scenarios/',name_scenario,'.txt'),'file');
    if(~existence)
        fileID = fopen(sstrcat(path,'/scenarios/',name_scenario,'.txt'),'w');
        fprintf(fileID,'%s',rawTextFromStorage);
        fclose(fileID);

        disp('DONE!')
    else
        adv = strcat('The file: /',name_scenario,' allready exist');
        b=questdlg(adv, 'Overwrite?','Yes','No','No');
        switch b
            case 'Yes'
                fileID = fopen(strcat(path,'/scenarios/',name_scenario,'.txt'),'w');
                fprintf(fileID,'%s',rawTextFromStorage);
                fclose(fileID);
                disp('DONE!')
        end
    end
    
    
end





