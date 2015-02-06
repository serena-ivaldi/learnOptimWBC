disp('GHC_RUNTIMEPARAM')
%% Constraints
constraints_list={'vellimit','vellimit','torquelimit','torquelimit'};
%cdata1 = [1;1];
cdata1 = [1;1000];
cdata2 = [0;1000];
cdata3 = [1;2000];
cdata4 = [0;2000];
constraints_data = [cdata1, cdata2, cdata3, cdata4];%, cdata5];


%% ChainedAlpha
transition_interval = 1.5;


%% Controller Parameters
epsilon = 0.002;
regularization = 0.01;
max_time = 2000;
