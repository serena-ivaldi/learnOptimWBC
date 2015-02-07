disp('UF_RUNTIMEPARAM')
% REPELLERS PARAMETERS
% GENERALIZE TO MULTICHAIN !!!
rep_obstacle_ref = [1 2]; % if i change the order of ref obstacle i change the order of repellor in the stacked case
% with this part i choose if for each repellers i want to use 3 or one
% activation policy if 1 only one if 0 we have three activation policy
single_alpha_chain1 = [1 1];
single_alpha_chain2 = [1];
single_alpha{1} = single_alpha_chain1;
single_alpha{2} = single_alpha_chain2;
type_of_rep_strct={'extended_combine','stacked' , 'extended_decoupled'};

%ALPHA PARAMETERS
%rbf
number_of_basis = 10;
redundancy = 3;
range = [0 , 12];
precomp_sample = false;
% value of theta that we have to change when we want to execute the result
% from the optimization step
%numeric_theta = [5.586974 12.000000 10.343330 11.936231 10.997103 12.000000 10.042855 11.247323 10.538352 9.181423 7.009495 0.000000 0.965310 0.160456 2.072685 2.753228 1.270566 2.130679 0.000000 4.426763 1.725728 2.665351 1.002842 2.293220 0.548551 2.927859 0.675315 1.330734 0.072510 3.967580 ];  
numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;


%CONTROLLER PARAMETERS
max_time = 100; %50
combine_rule = {'sum'}; % sum or projector
% with this term i introduce a damped least square structure inside my
% controller if regularizer is 0 i remove the regularizer action 
regularizer_chain_1 = [0 0]; 
regularized_chain_2 = [1];
regularizer{1} = regularizer_chain_1;
regularizer{2} = regularized_chain_2;


% CMAES PARAMETER
% starting value of parameters
%init_parameters = 6;
explorationRate =0.1;%[0, 1]
niter = 80;