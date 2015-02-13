disp('GHC_RUNTIMEPARAM')

%%%;;

%% Constraints
constraints_list={'vellimit','vellimit','torquelimit','torquelimit'};
%cdata1 = [1;1];
cdata1 = [1;1000];
cdata2 = [0;1000];
cdata3 = [1;2000];
cdata4 = [0;2000];
constraints_data = [cdata1, cdata2, cdata3, cdata4]%, cdata5];


%% flag to choose type of alpha 
% RBF or chained
choose_alpha = 'RBF';

%% ChainedAlpha
transition_interval = 1.5;

%% Alpha RBF
%ALPHA PARAMETERS
%rbf
number_of_basis = 10;
redundancy = 3;
value_range = [0 , 12];
precomp_sample = false;
% value of theta that we have to change when we want to execute the result
% from the optimization step

%from 10 generation of CMAES: collision with end-eff and table
numeric_theta = [1.351681 10.784147 9.724284 6.550806 7.740233 5.928500 8.123806 7.776163 6.548935 5.474038 7.455956 4.011111 6.704292 1.089315 3.712038 6.041540 5.098971 5.054418 6.312087 6.223340 ];
numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];


%% Controller Parameters
epsilon = 0.002;
regularization = 0.01;
max_time = 2000;


%% CMAES PARAMETER
% starting value of parameters
%init_parameters = 6;
explorationRate =0.1;%[0, 1]
niter = 80;  %number of generations
fitness = @fitness6;

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
