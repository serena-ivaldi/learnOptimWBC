function [t_, q, qd]=PlotCmaesResult(complete_path,input,name_scenario,time_struct,bestAction,bot1,learn_approach,varargin)
% conversion from rad to deg
RAD = 180/pi;
% for torque
interpolation_step = 0.001; 

% change the computation time more than i consider the execution a fail (useful whn i recompute the solution on another machine)
%controller.max_time = 1000;
% global variables for the function
simulator         = input{1}; % rbt or v-rep
controller = [];
time_sym_struct = [];
p = [];
pd = [];
if(strcmp(simulator,'rbt'))
    qinit             = input{2}; % initial position
    qdinit            = input{3}; % initial velocity
    time_sym_struct   = input{4}; %time struct for simulation with fixed step
    controller        = input{5}; % structure that contains every information about the specific instance of the problem
    fixed_step        = input{6}; % if is true i use ode4 (runge-kutta)
    torque_saturation = input{7};
    maxtime           = input{8};
    %options         = input{7}; % options
    
    % update of the parameters of activation functions and some reference
    % (if they are optimized)
    controller.UpdateParameters(bestAction.parameters)
    
    tic
    [t, q, qd] = DynSim(time_sym_struct,controller,qinit,qdinit,fixed_step,'TorqueSat',torque_saturation,'maxtime',maxtime);
    toc
    %toc(controller.current_time) for debugging the time deadline
    % generate cartesian position and cartesian velocity
    for ii =1:size(q{1},1) % generalize to multichain
        T =controller.subchains.sub_chains{1}.fkine(q{1}(ii,:)); % generalize to multichain
        p = [p;T(1:3,4)'];
        J = controller.subchains.sub_chains{1}.jacob0(q{1}(ii,:)); % generalize to multichain
        pd_cur = J*(qd{1}(ii,:))';% generalize to multichain
        pd = [pd ; pd_cur'];  
    end
elseif (strcmp(simulator,'icub_matlab'))
    % due to some change in the code i have to set the floating base
    % properties
    input{2}.active_floating_base = false;
    input{2}.numContacts = sum(input{2}.feet_on_ground,2);
    input{2}.contactLinkNames={'l_sole','r_sole'};
    % for some reason the icub robot does not work if  i save it in a file 
    % so i have to create it again
    bot2 = iCub(bot1.model_name);
    controller = input{4}; % structure that contains every information about the specific instance of the problem
    controller.visual_param.fc = [];
    % here i subistitute the icub with the new one
    bot1 = bot2;
    controller.subchains.whole_system = bot2;
    time_sym_struct = input{3};
    % update of the parameters of activation functions and some reference
    % (if they are optimized)
    controller.UpdateParameters(bestAction.parameters)
    
    tic
    [t, q, qd]=DynSim_iCub(controller,input{2});
    toc
    %toc(controller.current_time) for debugging the time deadline
    for ii =1:size(q,1) % generalize to multichain
        cur_q = q(ii,:);
        cur_qd = qd(ii,:);
        p_cur = bot1.offlineFkine(cur_q','r_hand'); % generalize to multichain
        p = [p;p_cur];
        J = bot1.offlineJacob0(cur_q','r_hand'); % generalize to multichain
        pd_cur = J*(cur_qd)';% generalize to multichain
        pd = [pd ; pd_cur'];  
    end
end
% compute the torque interpolated   
tau_=InterpTorque(controller,time_sym_struct,interpolation_step);
% copy best action inside the new folder
fileID = fopen(strcat(complete_path,'/','best_action.txt'),'wt');
fprintf(fileID,'%f ',bestAction.parameters);
fclose(fileID);

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
% plot of fitnes function
evolutions = size(bestAction.hist,2);
% remove failed mean and failed variance
% -10000000 is the penalty that is applied when i have a failure (not anymore to change)
index = 1;
evo = 1;
mean = 0;

if(strcmp(learn_approach,'CMAES'))
   for ww =1:evolutions
       % i discard the evolutions with failure final mean
       if(bestAction.hist(1,ww).performance>-1)
           listperformance = bestAction.hist(1,ww).listperformance;
           %remove all the failure from the computation of the variance
           listperformance = listperformance(listperformance~=-1);
           variance(index) = var(listperformance);
           mean(index) = bestAction.hist(1,ww).performance;
           evo(index) = index;
           index = index + 1; 
       end
   end
elseif(strcmp(learn_approach,'(1+1)CMAES'))  
  for ww =1:evolutions 
     if(bestAction.listperformance(ww)> -1 )
        %remove all the failure from the computation of the variance
        variance(index) = 0;
        mean= bestAction.listperformance(ww);
        evo(index) = index;
        index = index + 1;  
     end
  end
end

handle = figure;
hold on;
grid on;

plot(evo,mean);
% variance plot of cmaes not significant
% plot(evo, mean + 2 * variance, ':');
% plot(evo, mean - 2 * variance, ':');
xlab = xlabel('evolutions');
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab = ylabel('fitness');
 set(ylab,'FontSize',dim_lab,'Interpreter','latex');
%saveas(handle,strcat(complete_path,'/','fit'),'pdf');
%saveas(handle,strcat(complete_path,'/','fit'));
% plot 3d graph 
% handle = figure; hold on;
% text = LoadScenario(name_scenario);
% eval(text);
% grid on;
% % plot trace
% [ee,elbow] = ComputePositions(q{1},t_,controller);
% ee = ee';
% elbow = elbow';
% izy = 1;
% handle_vector = [];
% % ee_trajectory
% name_of_trace {1,izy} = 'end-effector';  
% handle1 = plot3(ee(:,1)',ee(:,2)',ee(:,3)','Color','r','LineWidth',2);
% handle_vector=[handle_vector,handle1];
% izy = izy + 1;
% % elbow_traj
% name_of_trace{1,izy} = 'elbow';  
% handle2 = plot3(elbow(:,1)',elbow(:,2)',elbow(:,3)','Color','g','LineWidth',2);
% handle_vector=[handle_vector,handle2];
% leg = legend(handle_vector,name_of_trace);
% set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','Best');
% saveas(handle,strcat(complete_path,'/','3d_traj'));
% if(strcmp(simulator,'rbt'))
%     bot1.plot(input{2});
% elseif(strcmp(simulator,'icub_matlab'))
%     bot1.plot(q,input{2});
% end
% saveas(handle,strcat(complete_path,'/','3d_traj_bot'));

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
%saveas(handle,strcat(complete_path,'/','e_e'),'pdf');
%saveas(handle,strcat(complete_path,'/','e_e'));
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
%saveas(handle,strcat(complete_path,'/','q'),'pdf');
%saveas(handle,strcat(complete_path,'/','q'));
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
%saveas(handle,strcat(complete_path,'/','tau'),'pdf');
%saveas(handle,strcat(complete_path,'/','tau'));

close all;

% build file for the execution on kinova
% WriteFF(p(:,1:3),3,strcat(complete_path,'/','cart_pos.txt'));
% WriteFF(pd(:,1:3),3,strcat(complete_path,'/','cart_vel.txt'));
% WriteFF(q{1},size(q{1},2),strcat(complete_path,'/','joint_pos.txt'));
% WriteFF(qd{1},size(qd{1},2),strcat(complete_path,'/','joint_vel.txt'));
% WriteFF(q{1}(1,:),size(q{1},2),strcat(complete_path,'/','start_joint_pos.txt'));
% WriteFF(p(1,1:3),3,strcat(complete_path,'/','start_cart_pos.txt'));


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





