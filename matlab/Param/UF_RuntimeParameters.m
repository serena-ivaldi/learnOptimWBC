disp('UF_RUNTIMEPARAM')

%%%;;

% REFERENCE PARAMETERS (this parameter only works if one of the specific trajectory has runtime parameters)
numeric_reference_parameter{1,1} = [0.047180 0.359539 1.045565 0.374223 -0.069047 0.013630 -0.495463 -0.131683 0.668327 -0.184017 1.115775 0.884010 0.120701 0.837400 1.189048]';


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
choose_alpha = 'RBF';  % RBF , constant

%RBF
number_of_basis = 5; %5; %10; %basis functions for the RBF
redundancy = 2; %3; %overlap of the RBF
value_range = [0 , 12];
precomp_sample = false;
% value of theta that we have to change when we want to execute the result
% from the optimization step
%numeric_theta = [0.068017 9.937933 10.629743 8.625690 4.620175 10.724682 6.943026 1.836172 6.005996 6.404127 1.499565 5.320011 5.059803 8.438304 2.319497 8.590403 9.120348 2.400932 9.071976 6.264097 ];
%numeric_theta =[0.068017 9.937933 10.629743 8.625690 4.620175 10.724682 6.943026 1.836172 6.005996 6.404127 1.499565 5.320011 5.059803 8.438304 2.319497 8.590403 9.120348 2.400932 9.071976 6.264097 ];
%numeric_theta =[2.3218    2.5695    6.8006    4.6558    5.7475    8.7383    3.5058    5.2817    6.9910    6.7590    4.5235    6.3875    7.3247    6.7258 8.5637];
numeric_theta =[3.314339 7.037574 8.010913 8.104115 10.420123 5.815695 10.639344 6.241454 1.656821 4.274706 10.098549 7.152070 0.000000 0.011717 0.000000];

% from sere 1
%numeric_theta = [5.819383 4.412794 5.286902 7.786384 7.599614 3.512520 5.989917 9.410994 7.444834 7.472545 4.532512 5.614148 7.970080 4.498142 6.194601 6.925731 4.815911 5.490313 5.294776 6.011380 ]

%from 10 generation of CMAES: collision with end-eff and table
%numeric_theta = [1.351681 10.784147 9.724284 6.550806 7.740233 5.928500 8.123806 7.776163 6.548935 5.474038 7.455956 4.011111 6.704292 1.089315 3.712038 6.041540 5.098971 5.054418 6.312087 6.223340 ];

% this is a good one (obtained by 80 generations of CMAES)
%numeric_theta = [2.885347 7.054374 6.510485 4.220996 3.779241 7.292772 6.753379 4.039816 3.503077 7.105706 7.242047 5.176997 6.656641 7.282674 6.310105 2.320801 6.164860 5.949270 5.958774 3.349248]; 

%numeric_theta = [2.718340 0.238570 4.959242 5.150985 10.810089 5.561797 6.436029 3.089579 7.488959 5.577574 5.300494 9.360753 5.395630 3.646393 5.427430 5.963953 10.538157 8.951330 7.672437 2.743474];
%numeric_theta = [3.493783 6.211959 7.883578 11.988846 7.900086 9.468388 6.525209 11.867391 7.355206 8.158990 0.000000 0.054878 11.131856 8.063698 1.871041 9.107188 3.646651 8.656589 11.419753 4.346246 ];  

%this is the task without the constraints of the table 
%numeric_theta =[12 12 12 12 12 12 12 12 12 12];

%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;
value_range_for_optimization_routine = [-0.5 , 1.5]; % this is a trick that im using to provide bound to the optimization procedure for parametric reference

%CONTROLLER PARAMETERS
max_time = 100; %50
combine_rule = {'sum'}; % sum or projector (with sum reppelers are removed)
% with this term i introduce a damped least square structure inside my
% controller if regularizer is 0 i remove the regularizer action 
% ONE FOR EACH TASK
regularizer_chain_1 = [0.001 0.001 0.001 0.001]; 
regularized_chain_2 = [1];
regularizer{1} = regularizer_chain_1;
regularizer{2} = regularized_chain_2;


% CMAES PARAMETER
% starting value of parameters
%init_parameters = 6;
explorationRate = 0.5; %0.1; %0.5; %0.1;%[0, 1]
niter = 2;  %number of generations
fitness = @fitness10;
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




