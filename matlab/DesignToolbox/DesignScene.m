%% When we design the scene is really important to check that each task
%% is in the reachibilty region of the robot. if it is not the case the
%% integration will take ages to finish

clear all
close all
clc

% in this variable we have to specify the name of the scenario:
% bot_scenario# where # is incremental
name_scenario = 'iCub_1';
% with this variable i decide when i want to save the designed scenario
save_now =true;


%% plot scene
list_of_kin_chain = {'trunk','left_arm','right_arm'};
feet_on_ground = [1 1];
plot_bot = iCub('model_arms_torso_free');
qjInit = plot_bot.InitializeState(list_of_kin_chain, feet_on_ground);
dqjInit     = zeros(plot_bot.ndof,1);
% icub starting velocity floating base
dx_bInit    = zeros(3,1);
omega_bInit = zeros(3,1);

% root reference link;
root_reference_link ='l_sole';

plot_bot.SetWorldFrameiCub(qjInit,dqjInit,dx_bInit,omega_bInit,root_reference_link);

[~,T_b,~,~] = plot_bot.GetState();

chiInit = [T_b; qjInit; plot_bot.dx_b; plot_bot.omega_W; dqjInit]';

%%%;;

hold on;axis equal;
%% modify from this point
global G_OB;

% plot sphere
% r = 0.15;
% x0 = -0.20; y0 = -0.286; z0 = 0.5;
% [x,y,z] = sphere(50);
% 
% x = x*r + x0;
% y = y*r + y0;
% z = z*r + z0;

%lightGrey = 0.8*[1 1 1]; % It looks better if the lines are lighter
%surface(x,y,z,'FaceColor', 'none','EdgeColor',lightGrey)


%%%;;
depth = 0.25;
width = 0.22;
center = -0.0681;
thickness = 0.01;
[X,Y,Z]=meshgrid(depth,center-width/2:0.001:center+width/2,0.45:0.001:1.0 );
%Y = -0.4*ones(1,size(X,1));

%Display the wall
% for i=1:size(X,3)
%     scatter3(X(:,:,i),Y(:,:,i),Z(:,:,i))
% end
Xmax = max(X(:)); Xmin = min(X(:));
Ymax = max(Y(:)); Ymin = min(Y(:));
Zmax = max(Z(:)); Zmin = min(Z(:));
if Xmax == Xmin
    Xmin = Xmin - thickness/2; Xmax = Xmax + thickness/2;
elseif Ymax == Ymin
    Ymin = Ymin - thickness/2; Ymax = Ymax + thickness/2;
elseif Zmax == Zmin
    Zmin = Zmin - thickness/2; Zmax = Zmax + thickness/2;
end

xyzpatch.vertices = [Xmax , Ymax , Zmax;
    Xmax , Ymin , Zmax;
    Xmin , Ymin , Zmax;
    Xmin , Ymax , Zmax;
    Xmax , Ymax , Zmin;
    Xmax , Ymin , Zmin;
    Xmin , Ymin , Zmin;
    Xmin , Ymax , Zmin;];

xyzpatch.faces   = [ 1 2 3 4;
    1 4 8 5;
    5 8 7 6;
    7 3 2 6;
    2 6 5 1;
    3 7 8 4];

lnkpatch = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceColor','red'); %'FaceAlpha',0.2,

r_e_e_point = [0.35,-0.15,0.7];
r_elbow_point = [0.24,-0.23,0.7];
scatter3(r_elbow_point(1,1),r_elbow_point(1,2),r_elbow_point(1,3),130,'b');
scatter3(r_e_e_point(1,1),r_e_e_point(1,2),r_e_e_point(1,3),130,'r');

% l_e_e_point = [0.35,0.0138,0.7];
% l_elbow_point = [0.24,0.0938,0.7];
% scatter3(l_elbow_point(1,1),l_elbow_point(1,2),l_elbow_point(1,3),130,'b');
% scatter3(l_e_e_point(1,1),l_e_e_point(1,2),l_e_e_point(1,3),130,'r');

% global obstacle
rapresentation.X = X(1,:,1);
rapresentation.Y = Y(:,1,1)';
rapresentation.Z = permute(Z(1,1,:),[2 3 1]);
ob1 = Obstacle(rapresentation,'wall',0.002);
G_OB = [ob1];

%%%EOF

params.sim_step = 0.01;
plot_bot.plot(chiInit,params);

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
