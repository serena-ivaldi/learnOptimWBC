disp('GHC_RUNTIMEPARAM')

%%%;;

%% Constraints
constraints_list={'vellimit','vellimit','torquelimit','torquelimit','obsavoid'}; %'obsavoid'

cdata1 = [1;1000];
cdata2 = [0;1000];
cdata3 = [1;2000];
cdata4 = [0;2000];
cdata5 = [1;3];
constraints_data = [cdata1, cdata2, cdata3, cdata4, cdata5];


%% flag to choose type of alpha 
% RBF or chaine
choose_alpha = 'RBF';

%% ChainedAlpha
transition_interval = 1.5;

%% Alpha RBF
%ALPHA PARAMETERS
%rbf
number_of_basis = 5;
redundancy = 2;
value_range = [0 , 12];
precomp_sample = false;
% value of theta that we have to change when we want to execute the result
% from the optimization step

%from 10 generation of CMAES: collision with end-eff and table
numeric_theta = [0.000000 6.112754 1.548512 4.320045 4.210416 3.384720 1.652093 2.600296 5.280306 2.041988 10.990456 11.266227 7.441896 12.000000 7.157816 0.598944 10.115051 11.252554 3.775318 3.506263  ];
%numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];


%% Controller Parameters
epsilon = 0.002;
regularization = 0.01;
max_time = 500;


%% CMAES PARAMETER
% starting value of parameters
%init_parameters = 6;
explorationRate =0.1;%[0, 1]
niter = 20;  %number of generations
fitness = @fitness7;

%% FITNESS PARAMETERS

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
