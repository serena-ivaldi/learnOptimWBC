clear all
close all
clc

% in this variable we have to specify the name of the scenario:
% bot_scenario# where # is incremental
name_scenario = 'jaco_scenario1';
% with this variable i decide when i want to save the designed scenario
save_now = false;


%% plot scene
plot_bot = MdlJaco();

%%%;;

hold on;axis equal;
%% modify from this point
global G_OB;

hold on;axis equal;
[X,Y,Z]=meshgrid(-0.05:0.001:0.3,-0.5,0.45:0.001:1.0);
%Y = -0.4*ones(1,size(X,1));
for i=1:size(X,2) 
    scatter3(X(:,:,i),Y(:,:,i),Z(:,:,i))
end
%elbow_point = [-0.1 -0.25 0.5];
e_e_point = [0,-0.63,0.70];
%intermediate_e_e_point = [ -0.3,-0.2,0.7];
%scatter3(elbow_point(1,1),elbow_point(1,2),elbow_point(1,3),130,'b');
scatter3(e_e_point(1,1),e_e_point(1,2),e_e_point(1,3),130,'b');
%scatter3(intermediate_e_e_point(1,1),intermediate_e_e_point(1,2),intermediate_e_e_point(1,3),130,'b');
% global obstacle
rapresentation.X = X;
rapresentation.Y = Y;
rapresentation.Z = Z;
ob1 = Obstacle(rapresentation,'wall',0.002);
G_OB = [ob1];

%%%EOF


plot_bot.plot(qr);
%plot_bot.teach();

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
