clear variable
close all
clc

%% IT DOESN'T WORK FOR MULTIPLE CHAINS

%% GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 20;
time_struct.step = 0.001;

time = time_struct.ti:time_struct.step:time_struct.tf;

number_of_kinematic_chain = 1;
number_of_action = 3; % this parameter define how many weigthing function i have to compute 
%% HandTuned 
starting_value = [0 1 0] ;
% inf means no transition but i have to add them because im using matrix
% so i have to keep the number of element even between each row and between each col
t1 = [13 inf;15 inf;0.5 3];
ti(:,:,1) = t1;
transition_interval1 = [0.5 0.5;1 0.5;0.5 1];
transition_interval(:,:,1) = transition_interval1;

hand_tuned_alphas = Alpha.HandTuneAlpha.BuildCellArray(number_of_kinematic_chain,number_of_action,starting_value,ti,transition_interval,time_struct);
%% RBF
number_of_basis = 5; %5; %10; %basis functions for the RBF
redundancy = 2; %3; %overlap of the RBF
value_range = [0 , 12];
precomp_sample = false;
numeric_theta = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
alphas = Alpha.RBF.BuildCellArray(number_of_kinematic_chain,number_of_action,time_struct,number_of_basis,redundancy,value_range,precomp_sample,numeric_theta,false);

%% Learning Phase

% build X matrix
X = [];
for j = 1:number_of_action
   X = [X,time'];
end

% build training set Y matrix
Y_col = zeros(length(time),1);
Y=[];
for j = 1:number_of_action
    i=1;
    for t = time  
       Y_col(i,1) = hand_tuned_alphas{j}.GetValue(t); 
       i=i+1;    
    end 
    Y = [Y ,Y_col];
end
theta = [];
for k = 1 : number_of_action
   cur_theta = alphas{k}.LearningFromDemo(X(:,k),Y(:,k)); 
   theta = [theta,cur_theta'];
end








