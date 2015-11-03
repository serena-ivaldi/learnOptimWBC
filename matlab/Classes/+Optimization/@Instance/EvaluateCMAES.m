
%%***************************************************
%% Task dependent evaluation function
%%***************************************************


%% 
%data2save             this is a structure used to collect data for visualization or debugging purposes



function [performance, succeeded, data2save] = EvaluateCMAES(obj,action,cur_candidates_index,ismean)
%%
%% action is a row vector where each element denotes the amplitude of one
%% Gaussian Kernel
%%
%% performance denotes the mean squared error to the one dimensional
%% target function

% i have to assign the variable to avoid error generation
data2save = [];

 %try
    disp('i am in evaluate CMAES')
    action
    [output]=obj.run(action);
    
    succeeded = 1;
    
    %tic
    % insert fitness function 
    performance = feval(obj.fitness,obj,output)
    %toc
    
    %% DO NOT CHANGE THIS PART!
    if(obj.constraints)
       % here i compute the final penalty value for each candidate of the
       % current population
       obj.penalty_handling.ComputeConstraintsViolation(cur_candidates_index)
       if(cur_candidates_index < 0)
          % here im going to save the average perfomance without correction
          data2save.performance = performance;
          % perfomance with correction
          performance = performance - obj.penalty_handling.fitness_penalties(1);
       end
    end
    %%
    
    % cancel all the information relative to the current iteration (control action)
    feval(obj.clean_function,obj);
    
% catch err
%      disp('i am in evaluate CMAES error side')
%      % cancel all the information relative to the current iteration (control action)
%      feval(obj.clean_function,obj);
%      succeeded = 0;
%      performance = -1;
%  end


end