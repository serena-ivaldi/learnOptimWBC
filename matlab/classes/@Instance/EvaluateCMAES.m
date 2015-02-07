
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
    disp('i am in evaluate CMAES')
    action
    [t, q, qd]=obj.run(action);
    
    succeeded = 1;
    
    %tic
    % insert fitness function 
    performance = feval(obj.fitness,obj,t,q)
    %toc
    
    % cancel all the information relative to the current iteration (control action)
    obj.controller.CleanTau();
    
catch err
    disp('i am in evaluate CMAES error side')
    obj.controller.CleanTau(); 
    succeeded = 0;
    performance = -10000000;
    
end


end