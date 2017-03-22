%% all the function from this point DO NOT SUPPORT multichain structure
%% i compute everything only for the first chain



classdef  Instance
    
   properties
      penalty_handling % object to handle penalties inside the optimization routine
      learn_procedure  % string that specify which learning method im going to use
      constraints      % flag that activates or deactivates the constraints handling (true: constraints active, false: constraints not active) 
      run_function     % function called in run specific for each optimization problem
      fitness          % fitness function handle
      clean_function   % function called to do some stuff after using the run function (optionally could be empty)
      input_4_run      % this variable is a cell array that contains the data that are needed to execute the run function
      fitness_result   % in this vector i save the value of the fitness function 
      data2save        % in this structure im going to save all the data that i need for visualization / debugging purpose
   end
       
   methods
       function obj = Instance(penalty_handling,learn_procedure,run_function,fitness,clean_function,input_4_run)
           if(isempty(penalty_handling))
              obj.constraints = false;
              penalty_handling.EvaluateConstraints=@(input_,iteration_)DoNothing(input_,iteration_);
              penalty_handling.n_constraint = 0;
           else
              obj.constraints = true;
           end
           obj.learn_procedure = learn_procedure;
           obj.penalty_handling = penalty_handling;
           obj.run_function = run_function;  
           obj.fitness = fitness;
           obj.input_4_run = input_4_run;
           obj.clean_function = clean_function;
       end
       
       % this function has to give back something that let me compute the
       % fitness function for that sample
       function [output]=run(obj,parameters)
            %disp('im in run')   
            [output]=feval(obj.run_function,obj,parameters);
       end
       
       function [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded, G_data2save]=CMAES(obj,num_of_param,start_action,niter,explorationRate,cmaes_value_range)
          %Parameter space
          NumParam = num_of_param;
          % start_value for action
          settings.action = start_action;
          if(iscell(cmaes_value_range))
             settings.minAction = cmaes_value_range{1};
             settings.maxAction = cmaes_value_range{2};  
          elseif(isvector(cmaes_value_range))
             settings.minAction = ones(1,NumParam).*cmaes_value_range(1,1);
             settings.maxAction = ones(1,NumParam).*cmaes_value_range(1,2);
          else
             error('something wrong with cmaes_value_range')
          end
          %CMA-ES settings
          settings.nIterations = niter;     
          settings.explorationRate = explorationRate; %[0, 1]
          settings.fnForwardModel = @(obj_,a_,curr_candidate_,ismean_)EvaluateCMAES(obj_,a_,curr_candidate_,ismean_);
          settings.plotState = 1;         %{0,1} plot offsprings yes no
          %search optimal parameters 
          if(strcmp(obj.learn_procedure,'CMAES'))
            [mean_performances, bestAction, BestActionPerEachGen, policies,costs, succeeded,G_data2save] = obj.LearnCMAES(settings);
          elseif(strcmp(obj.learn_procedure,'(1+1)CMAES'))
            [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded,G_data2save] = obj.Learn1plus1CMAES(settings);
          elseif(strcmp(obj.learn_procedure,'CEM'))
            [mean_performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = obj.CEM(settings);
          elseif(strcmp(obj.learn_procedure,'BO'))
              [mean_performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = obj.BO(settings);
          elseif(strcmp(obj.learn_procedure,'BO(1+1)CMAES'))
              [mean_performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = obj.BO1plus1CMAES(settings);
          end
          figure;
          plot(mean_performances);      
       end
      
       function input_vec = CreateInputFromParameters(obj,parameters) 
            input_vec = repmat({parameters},1,obj.penalty_handling.n_constraint);  
       end
       
       % to have a common interface with Objproblem im introducing a
       % function to evaluate constraints at some point
       function [c, ceq,performance] = computeConstr(obj,input)
            try
               [c, ceq,performance] = obj.computConstrViol(input);
            catch err
                disp('computeConstr failed');
            end
       end
       
       % inner function to have a common interface with Objproblem im introducing a
       % function to evaluate constraints at some point
       %% TODO check it
       function [c, ceq,performance] = computConstrViol(obj,input)                    
            % each time i have to compute this 
            try
                disp('i am in computConstrViol fitness computation')
                [output]=obj.run(input);
                
                performance = feval(obj.fitness,obj,output);
                %toc
                
                %% DO NOT CHANGE THIS PART!
                if(obj.constraints)
                   c_index = -1; %i'm just considering one candidate (cf. FixPenalty class)
                   obj.penalty_handling.ComputeConstraintsViolation(c_index);
                   c = [];
                   ceq = [];
                   if(strcmp(obj.learn_procedure ,'CMAES'))
                      % perfomance with correction
                      performance = performance - obj.penalty_handling.fitness_penalties(1);
                   end
                end
                
                for i=1:length(obj.penalty_handling.constraints_type)
                    if  obj.penalty_handling.constraints_type(i)
                        c = [c; obj.penalty_handling.penalties(1,i)];
                    else
                        ceq = [ceq; obj.penalty_handling.penalties(1,i)];
                    end
                end 
                % i need to check the emptyness to be sure that im giving back
                % something meanigfull
                if(isempty(c))
                    c = 0;
                end
                if(isempty(ceq))  
                    ceq = 0;
                end
                % cancel all the information relative to the current iteration (control action)
                feval(obj.clean_function,obj);
            catch err
                disp('intergration error in  ComputeConstraintViolation and perfomance')
                performance = -1; %penalty if the computation of the fitness failed
                c_index = -1; %i'm just considering one candidate (cf. FixPenalty class)
                obj.penalty_handling.ComputeConstraintsViolation(c_index);
                c = [];
                ceq = [];
                for i=1:length(obj.penalty_handling.constraints_type)
                    if  obj.penalty_handling.constraints_type(i)
                        c = [c; obj.penalty_handling.penalties(1,i)];
                    else
                        ceq = [ceq; obj.penalty_handling.penalties(1,i)];
                    end
                end 
                % i need to check the emptyness to be sure that im giving back
                % something meanigfull
                if(isempty(c))
                    c = 0;
                end
                if(isempty(ceq))  
                    ceq = 0;
                end
                % cancel all the information relative to the current iteration (control action)
                feval(obj.clean_function,obj);
            end 
           
       end
       
   end
    
end
