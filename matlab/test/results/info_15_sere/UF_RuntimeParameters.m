disp('UF_RUNTIMEPARAM')

%%%;;

% REPELLERS PARAMETERS
% GENERALIZE TO MULTICHAIN !!!
rep_obstacle_ref = [1 2]; % if i change the order of ref obstacle i change the order of repellor in the stacked case
J_damp = 0.01;
% with this part i choose if for each repellers i want to use 3 or one
% activation policy if 1 only one if 0 we have three activation policy
single_alpha_chain1 = [1 1];
single_alpha_chain2 = [1];
single_alpha{1} = single_alpha_chain1;
single_alpha{2} = single_alpha_chain2;
type_of_rep_strct={'extended_decoupled' 'extended_combine','stacked' };

%ALPHA PARAMETERS
%rbf
number_of_basis = 10; %3; %5; %10; %basis functions for the RBF
redundancy = 2; %3; %overlap of the RBF
value_range = [0 , 12];
precomp_sample = false;
% value of theta that we have to change when we want to execute the result
% from the optimization step

%from 10 generation of CMAES: collision with end-eff and table
numeric_theta = [1.351681 10.784147 9.724284 6.550806 7.740233 5.928500 8.123806 7.776163 6.548935 5.474038 7.455956 4.011111 6.704292 1.089315 3.712038 6.041540 5.098971 5.054418 6.312087 6.223340 ];

% this is a good one (obtained by 80 generations of CMAES)
%numeric_theta = [2.885347 7.054374 6.510485 4.220996 3.779241 7.292772 6.753379 4.039816 3.503077 7.105706 7.242047 5.176997 6.656641 7.282674 6.310105 2.320801 6.164860 5.949270 5.958774 3.349248]; 

%numeric_theta = [2.718340 0.238570 4.959242 5.150985 10.810089 5.561797 6.436029 3.089579 7.488959 5.577574 5.300494 9.360753 5.395630 3.646393 5.427430 5.963953 10.538157 8.951330 7.672437 2.743474];
%numeric_theta = [3.493783 6.211959 7.883578 11.988846 7.900086 9.468388 6.525209 11.867391 7.355206 8.158990 0.000000 0.054878 11.131856 8.063698 1.871041 9.107188 3.646651 8.656589 11.419753 4.346246 ];  

%this is the task without the constraints of the table 
%numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;


%CONTROLLER PARAMETERS
max_time = 100; %50
combine_rule = {'projector'}; % sum or projector
% with this term i introduce a damped least square structure inside my
% controller if regularizer is 0 i remove the regularizer action 
regularizer_chain_1 = [0.01 0]; 
regularized_chain_2 = [1];
regularizer{1} = regularizer_chain_1;
regularizer{2} = regularized_chain_2;


% CMAES PARAMETER
% starting value of parameters
%init_parameters = 6;
explorationRate = 0.01; %0.1; %0.5; %0.1;%[0, 1]
niter = 80;  %number of generations
fitness = @fitness6;

% FITNESS PARAMETERS

%%%EOF

%% DO NOT MODIFY THIS PART 


% i read the parameters from this file
rawTextFromStoragePart = fileread(which(mfilename));
rawTextFromStoragePart = regexp(rawTextFromStoragePart,['%%%;;' '(.*?)%%%EOF'],'match','once');

% i read the parameters from fitness function
name_fitness = func2str(fitness);
rawTextFromStorageIsntace = fileread(which(name_fitness));
rawTextFromStorageIsntace = regexp(rawTextFromStorageIsntace,['%%%;;' '(.*?)%%%EOF'],'match','once');

% attach the info on the fitness function to rawTextFromStoragePart
rawTextFromStoragePart = strcat(rawTextFromStoragePart,rawTextFromStorageIsntace);




