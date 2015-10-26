%% all the function from this point DO NOT SUPPORT multichain structure
%% i compute everything only for the first chain



classdef  Instance
    
   properties
      penalty_handling % object to handle penalties inside the optimization routine
      constraints      % flag that activates or deactivates the constraints handling (true: constraints active, false: constraints not active) 
      run_function     % function called in run specific for each optimization problem
      fitness          % fitness function handle
      clean_function   % function called to do some stuff after using the run function (optional could be empty)
      input_4_run      % this variable is a cell array that contains the data that are needed to execute the run function
      fitness_result   % in this vector i save the value of the fitness function 
      
   end
       
    
   methods
       
       function obj = Instance(penalty_handling,constraints,run_function,fitness,clean_function,input_4_run)
           obj.penalty_handling = penalty_handling;
           obj.constraints = constraints;
           obj.run_function = run_function;  
           obj.fitness = fitness;
           obj.input_4_run = input_4_run;
           obj.clean_function = clean_function;
         
       end
       
       % this function has to give back something that let me compute the
       % fitness function for that sample
       function [output]=run(obj,parameters)
            disp('im in run')   
            [output]=feval(obj.run_function,obj,parameters);
       end
       
       function [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded]=CMAES(obj,num_of_param,start_action,niter,explorationRate,cmaes_value_range)
          %Parameter space
          NumParam = num_of_param;
          % start_value for action
          settings.action = start_action;
          settings.minAction = ones(1,NumParam).*cmaes_value_range(1,1);
          settings.maxAction = ones(1,NumParam).*cmaes_value_range(1,2);


          %CMA-ES settings
          settings.nIterations = niter;     
          settings.explorationRate = explorationRate; %[0, 1]
          settings.fnForwardModel = @(obj_,a_,curr_candidate_,ismean_)EvaluateCMAES(obj_,a_,curr_candidate_,ismean_);
          settings.plotState = 1;         %{0,1} plot offsprings yes no

          %search optimal parameters
          [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded] = obj.LearnCMAES(settings);

          figure;
          plot(mean_performances);      
      end
       
       
       
       
       
       
   end
    
end