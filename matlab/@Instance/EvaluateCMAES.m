
%%***************************************************
%% Task dependent evaluation function
%%***************************************************
function [performance succeeded] = EvaluateCMAES(action, isMean,obj)
%%
%% action is a row vector where each element denotes the amplitude of one
%% Gaussian Kernel
%%
%% performance denotes the mean squared error to the one dimensional
%% target function
succeeded = 1;

obj.run(action)

%fitness function (TO FIX)
performance = exp(-20*sum((tFn - aFn).^2)/length(x));%-sum((tFn - aFn).^2)/100; 

end