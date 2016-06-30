%% When we design the scene is really important to check that each task
%% is in the reachibilty region of the robot. if it is not the case the
%% integration will take ages to finish

clear all
close all
clc

% in this variable we have to specify the name of the scenario:
% bot_scenario# where # is incremental
name_scenario = 'humanoids_rob_benck_1';
% with this variable i decide when i want to save the designed scenario
save_now =true;


%% plot scene
plot_bot = MdlLBR4pSimple();

%%%;;

hold on;axis equal;
%% modify from this point
global G_OB;

% plot sphere
r = 0.15;
x0 = -0.20; y0 = -0.286; z0 = 0.5;
[x,y,z] = sphere(50);

x = x*r + x0;
y = y*r + y0;
z = z*r + z0;

lightGrey = 0.8*[1 1 1]; % It looks better if the lines are lighter
surface(x,y,z,'FaceColor', 'none','EdgeColor',lightGrey)


%%%;;
[X,Y,Z]=meshgrid(-0.05:0.001:0.3,-0.5,0.45:0.001:1.0);
%Y = -0.4*ones(1,size(X,1));
for i=1:size(X,2) 
    scatter3(X(:,:,i),Y(:,:,i),Z(:,:,i))
end
wrist_point = [x0,y0,z0] + [-0.124388 -0.137898 0.133180];
e_e_point = [-0.022,-0.722,0.709];
%intermediate_e_e_point = [ -0.3,-0.2,0.7];
scatter3(wrist_point(1,1),wrist_point(1,2),wrist_point(1,3),130,'b');
scatter3(e_e_point(1,1),e_e_point(1,2),e_e_point(1,3),130,'b');
%scatter3(intermediate_e_e_point(1,1),intermediate_e_e_point(1,2),intermediate_e_e_point(1,3),130,'b');
% global obstacle
rapresentation.X = X(1,:,1);
rapresentation.Y = Y(:,1,1)';
rapresentation.Z = permute(Z(1,1,:),[2 3 1]);
ob1 = Obstacle(rapresentation,'wall',0.002);
G_OB = [ob1];


% plot_subchain1 = [7];
% plot_target_link{1} = plot_subchain1;
% % reference parameters
% traj_type = {'cartesian'};
% control_type = {'x'};
% type_of_traj = {'func'};
% geometric_path = {'circular'};
% time_law = {'linear'};
% %parameters first chains
% geom_parameters{1,1} = [0.3 pi/2 pi/2 0 0.30 -0.75 0.5]; % regulation
% dim_of_task{1,1}={[1;1;1]}; 
% plot_time_struct.ti = 0;
% plot_time_struct.tf = 10;
% plot_time_struct.step = 0.1;
% plot_dim_of_task{1,1}={[1;1;1]};
% 
% %% reference
% % if type_of_task = sampled i have to specify the Time to reach the
% % end of the trajectories that is equal to the simulation time
% plot_reference = References(plot_target_link,traj_type,control_type,geometric_path,geom_parameters,time_law,plot_time_struct,plot_dim_of_task,type_of_traj);
% plot_reference.BuildTrajs();
% 
% 
% p_tot=[];
% for t=plot_time_struct.ti:plot_time_struct.step:plot_time_struct.tf
%  
% 	p_cur=plot_reference.GetTraj(1,1,t);
% 	p_tot = [p_tot,p_cur];
% 
% end
%   plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));



% hold on;axis equal;
% [X,Y,Z]=meshgrid(-0.05:0.001:0.3,-0.5,0.45:0.001:1.0);
% %Y = -0.4*ones(1,size(X,1));
% for i=1:size(X,2) 
%     scatter3(X(:,:,i),Y(:,:,i),Z(:,:,i))
% end
% wrist_point = [-0.309 -0.469 0.581];
% e_e_point = [0,-0.63,0.70];
% %intermediate_e_e_point = [ -0.3,-0.2,0.7];
% scatter3(wrist_point(1,1),wrist_point(1,2),wrist_point(1,3),130,'b');
% scatter3(e_e_point(1,1),e_e_point(1,2),e_e_point(1,3),130,'b');
% %scatter3(intermediate_e_e_point(1,1),intermediate_e_e_point(1,2),intermediate_e_e_point(1,3),130,'b');
% % global obstacle
% rapresentation.X = X;
% rapresentation.Y = Y;
% rapresentation.Z = Z;
% ob1 = Obstacle(rapresentation,'wall',0.002);
% G_OB = [ob1];

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
        fileID = fopen(strcat(path,'/scenarios/',name_scenario,'.txt'),'w');
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
