clear all
close all
clc

% in this variable we have to specify the name of the scenario:
% bot_scenario# where # is incremental
name_scenario = 'lbr_scenario11';
% with this variable i decide when i want to save the designed scenario
save_now = false;


%% plot scene
plot_bot = MdlLBR4p();

%%%;;
global G_OB;

hold on;axis equal;
%% modify from this point
plot_subchain1 = [7];
plot_target_link{1} = plot_subchain1;
% reference parameters
plot_type = {'cartesian_x'};
plot_control_type = {'tracking'};
plot_type_of_traj = {'func'};
plot_traj = {'elastic'};
plot_time_law = {'linear'};


%parameters first chains
plot_geom_parameters{1,1} = [5 2 1];
plot_time_struct.ti = 0;
plot_time_struct.tf = 20;
plot_time_struct.step = 0.01;

plot_dim_of_task{1,1}={[1;1;1]};

%% reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
plot_reference = References(plot_target_link,plot_type,plot_control_type,plot_traj,plot_geom_parameters,plot_time_law,plot_time_struct,plot_dim_of_task,plot_type_of_traj);
plot_reference.BuildTrajs();

plot_reference.cur_param_set{1,1} = [0.7 0.6 0.5 0.4 0.3 -0.3 0.0 -0.2 0.0 0.1 0.1 0.5 0.4 0.3 0.2];
plot_reference.cur_param_set{1,1} = plot_reference.cur_param_set{1,1}';

p_tot=[];
for t=plot_time_struct.ti:plot_time_struct.step:plot_time_struct.tf
 
	p_cur=plot_reference.GetTraj(1,1,t);
	p_tot = [p_tot,p_cur];

end

plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));

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
