%% kinematic simulation of jaco to asses the trajectory

clear variables
close all
clc

% Integral action 
P = 10;
%damping factor 
lambda = 0.001;

fps = 1000;

% Jaco model
plot_bot = MdlJaco();

%% read data from file
load('ground_truth.mat');

start_joint_pos=[];
fid = fopen('start_joint_pos.txt');
tline = fgetl(fid); %discard first line 
tline = fgetl(fid);
tlin = strsplit(tline);
for i=1:size(tlin,2)
start_joint_pos = [start_joint_pos,str2double(tlin{1,i})];
end


%% compute the final time 


% the idea is that because i suppose that the time step is fixed to 0.001 
% i can know the time in second by simply mult the time step for the number
% of element
% this is useful because when i use the slowdown function i can forget the
% final time
plot_time_struct.ti = 0;
plot_time_struct.step = 0.001;
plot_time_struct.tf = round(size(p,1)*plot_time_struct.step);


%% compute joint position 
cur_joint = start_joint_pos';
q = [];
index = 1;
tot_index = size(p,1);
for t = plot_time_struct.ti:plot_time_struct.step:plot_time_struct.tf
   cur_cart = plot_bot.fkine(cur_joint);
   cur_cart = cur_cart(1:3,4);
   J =plot_bot.jacob0(cur_joint);
   J = J(1:3,1:end);
   I=eye(3,3);
   J_damp = J'/(J*J' + I*lambda);
   qd = J_damp*(P*(p(index,:)' - cur_cart) + pd(index,:)');
   %qd = pinv(J)*pd(index,:)';
   cur_joint = cur_joint + (qd*plot_time_struct.step);
   q = [q , cur_joint];
   
   percent = (index/tot_index)*100;
   disp(percent)
   
   index = index + 1;
end

%% plot result
plot3(p(:,1),p(:,2),p(:,3));
plot_bot.plot(q','fps',fps)