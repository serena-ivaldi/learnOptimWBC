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
numeric_theta = [8.549245 2.205698 4.116592 1.417148 0.910537 1.069052 1.660022 0.674840 3.667179 3.980646 9.175454 5.460261 0.894753 2.923977 3.848606 5.023752 1.811986 5.588147 2.391248 2.893621 9.357446 1.011828 2.018193 3.509277 2.664418 1.194698 1.581755 4.310963 2.616728 3.807974 4.077435 6.794821 11.250924 2.119580 10.315526 10.829058 8.772832 3.834187 4.986195 9.721260 4.458052 11.855490 6.566780 5.641343 8.795516 11.002145 8.443036 5.658131 4.960512 6.806842 11.167998 11.273477 10.726782 10.736765 10.931710 0.535674 11.777257 10.651311 11.068113 9.945272 9.575991 4.687163 11.327296 10.984750 10.178523 3.318329 4.074540 11.286607 10.529069 10.660712 8.956185 1.873538 4.672742 1.253890 3.482083 4.687677 10.650933 4.854486 9.480859 3.951258 11.307175 11.485823 10.767789 11.113316 7.333339 11.412319 11.665979 10.533045 4.767150 1.124154 ];
%numeric_theta =[12 12 12 12 12 12 12 12 12 12 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];


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
