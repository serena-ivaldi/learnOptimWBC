function complete_path = PlotSingleExecResult(name_folder,q,qd,t_,time_sym_struct,controller,time_struct,name_scenario,bot1)
% conversion from rad to deg
RAD = 180/pi;
% for torque
interpolation_step = 0.001;
% generate cartesian position and cartesian velocity
p=[];
pd=[];
for iij =1:size(q{1},1) % generalize to multichain
   T =controller.subchains.sub_chains{1}.fkine(q{1}(iij,:)); % generalize to multichain
   p = [p;T(1:3,4)'];
   J = controller.subchains.sub_chains{1}.jacob0(q{1}(iij,:)); % generalize to multichain
   pd_cur = J*(qd{1}(iij,:))';% generalize to multichain
   pd = [pd ; pd_cur'];  
end
% compute the torque interpolated   
tau_=InterpTorque(controller,time_sym_struct,interpolation_step);

% create folder 
allpath=which('FindData.m');
path=fileparts(allpath);
complete_path = strcat(path,'/results/single_executions/',name_folder);
mkdir(complete_path)

% plot parameters
dim_tit=16;
dim_lab=14;
dim_leg=12;
% plot alpha 
time = time_struct.ti:time_struct.step:time_struct.tf;
vec_values = zeros(size(time));
handle_vec = [];
for ii = 1:size(controller.alpha,1)
    for jj = 1:size(controller.alpha,2)         
        i=1;
        for t = time
            vec_values(i) = controller.alpha{ii,jj}.GetValue(t); 
            i=i+1;
        end
        handle_vec(ii,jj) = figure;
        plot(time,vec_values);
        grid on;
        xlab = xlabel('t [s]');
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab = ylabel(strcat('\alpha_{',num2str(ii),num2str(jj),'}'));
        set(ylab,'FontSize',dim_lab);      
    end
end
% save alpha plot fig and jpg
for ii = 1:size(controller.alpha,1)
    for jj = 1:size(controller.alpha,2)
      saveas(handle_vec(ii,jj),strcat(complete_path,'/','alpha',num2str(ii),num2str(jj)),'pdf');
      saveas(handle_vec(ii,jj),strcat(complete_path,'/','alpha',num2str(ii),num2str(jj)));
    end
end
% plot 3d graph 
handle = figure; hold on;
text = LoadScenario(name_scenario);
eval(text);
grid on;
% plot trace
[ee,elbow] = ComputePositions(q{1},t_,controller);
ee = ee';
elbow = elbow';
izy = 1;
handle_vector = [];
% ee_trajectory
name_of_trace {1,izy} = 'end-effector';  
handle1 = plot3(ee(:,1)',ee(:,2)',ee(:,3)','Color','r','LineWidth',2);
handle_vector=[handle_vector,handle1];
izy = izy + 1;
% elbow_traj
name_of_trace{1,izy} = 'elbow';  
handle2 = plot3(elbow(:,1)',elbow(:,2)',elbow(:,3)','Color','g','LineWidth',2);
handle_vector=[handle_vector,handle2];
leg = legend(handle_vector,name_of_trace);
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','Best');
saveas(handle,strcat(complete_path,'/','3d_traj'));
bot1.plot(q{1}(1,:));
saveas(handle,strcat(complete_path,'/','3d_traj_bot'));

% plot x y z
handle = figure;
plot(p(:,1:3));
grid on;
leg = legend('X','Y','Z');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','best');
xlab=xlabel('time [ms]'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('cartesian position [m]'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');
saveas(handle,strcat(complete_path,'/','e_e'),'pdf');
saveas(handle,strcat(complete_path,'/','e_e'));
%plot q 
handle = figure;
plot(q{1}*RAD);
grid on;
xlab=xlabel('time [ms]'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('joints position [deg]'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');
for i = 1:size(q{1},2)
   str_name{i} = strcat('q_{',num2str(i),'}');
end
leg=legend(str_name);
set(leg,'FontSize',dim_leg,'Location','best');
saveas(handle,strcat(complete_path,'/','q'),'pdf');
saveas(handle,strcat(complete_path,'/','q'));
% plot tau
handle = figure;
plot(tau_)
grid on;
xlab=xlabel('time [ms]'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('joints torque [N*m]'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');
for i = 1:size(q{1},2)
   str_name1{i} = strcat('U_{',num2str(i),'}');
end
leg=legend(str_name1);
set(leg,'FontSize',dim_leg,'Location','best');
saveas(handle,strcat(complete_path,'/','tau'),'pdf');
saveas(handle,strcat(complete_path,'/','tau'));
hold off;

% build file for the execution on kinova
WriteFF(p(:,1:3),3,strcat(complete_path,'/','cart_pos.txt'));
WriteFF(pd(:,1:3),3,strcat(complete_path,'/','cart_vel.txt'));
WriteFF(q{1},size(q{1},2),strcat(complete_path,'/','joint_pos.txt'));
WriteFF(qd{1},size(qd{1},2),strcat(complete_path,'/','joint_vel.txt'));
WriteFF(q{1}(1,:),size(q{1},2),strcat(complete_path,'/','start_joint_pos.txt'));
WriteFF(p(1,1:3),3,strcat(complete_path,'/','start_cart_pos.txt'));


% figure
% alpha.PlotBasisFunction();
% 
% spani = 0:0.001:range(1,2);
% i = 1;
% sigvalue = zeros(1,size(spani,2));
% for x=spani
%     t = 0;
%     numeric_theta = x*ones(number_of_basis,1);
%     sigvalue(i) = feval(alpha.func,t,numeric_theta);
%     i= i + 1;
% end
% figure
% plot(spani,sigvalue)

end




% i obtain the interpolated torque and in this way i can compute mean and
% variance
function torque=InterpTorque(controller,time_struct,interp_step)
   time = time_struct.ti:interp_step:time_struct.tf;
   if(size(controller.torques_time{1},2) == size(controller.torques{1},2))
      [unique_time,ia,ic] = unique(controller.torques_time{1});
   else
      diff = size(controller.torques_time{1},2) - size(controller.torques{1},2) + 1;
      controller.torques_time{1} = controller.torques_time{1}(diff:end);
      [unique_time,ia,ic] = unique(controller.torques_time{1});
   end
   torque = [];
   for i = 1:size(controller.torques{1},1)
      unique_torque = controller.torques{1}(i,ia);
      interp_torque = interp1(unique_time,unique_torque,time,'nearest');
      torque = [torque , interp_torque'];
   end 
end
