
%%***************************************************
%% Task dependent evaluation function
%%***************************************************
function [performance succeeded] = EvaluateCMAES(action,obj)
%%
%% action is a row vector where each element denotes the amplitude of one
%% Gaussian Kernel
%%
%% performance denotes the mean squared error to the one dimensional
%% target function
succeeded = 1;

try
    obj.run(action)
catch err
    perfomance = 0;
end
%fitness function i have to change the structure of the feval
performance = feval(obj.fitness,1);

end