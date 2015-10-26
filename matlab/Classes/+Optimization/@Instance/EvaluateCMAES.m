
%%***************************************************
%% Task dependent evaluation function
%%***************************************************
function [performance succeeded] = EvaluateCMAES(obj,action,cur_candidates_index,ismean)
%%
%% action is a row vector where each element denotes the amplitude of one
%% Gaussian Kernel
%%
%% performance denotes the mean squared error to the one dimensional
%% target function


 try
    disp('i am in evaluate CMAES')
    action
    [output]=obj.run(action);
    
    succeeded = 1;
    
    %tic
    % insert fitness function 
    performance = feval(obj.fitness,obj,output)
    %toc
    
    %% DO NOT CHANGE THIS PART!
    % here i compute the final penalty value for each candidate of the
    % current population
    obj.penalty_handling.ComputeConstraintsViolation(cur_candidates_index)
    if(cur_candidates_index == -1)
        performance = performance - obj.penalty_handling.fitness_penalties(1);
    end
    %%
    
    % cancel all the information relative to the current iteration (control action)
    feval(obj.clean_function,obj);
    
catch err
     disp('i am in evaluate CMAES error side')
     % cancel all the information relative to the current iteration (control action)
     feval(obj.clean_function,obj);
     succeeded = 0;
     performance = -1;
 end


end