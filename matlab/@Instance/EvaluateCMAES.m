
%%***************************************************
%% Task dependent evaluation function
%%***************************************************
function [performance succeeded] = EvaluateCMAES(obj,action,ismean)
%%
%% action is a row vector where each element denotes the amplitude of one
%% Gaussian Kernel
%%
%% performance denotes the mean squared error to the one dimensional
%% target function


try
    obj.run(action)
    
    succeeded = 1;
    
    % insert fitness function 
    performance = rand();
    
    % cancel all the information relative to the current iteration (control action)
    obj.controller.CleanTau();
    
catch err
    
    succeeded = 0;
    performance = 0;
    
end


end