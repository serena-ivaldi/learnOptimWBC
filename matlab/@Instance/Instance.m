%% all the function from this point DO NOT SUPPORT multichain structure
%% i compute everything only for the first chain



classdef  Instance
    
   properties
      controller      % structure that contains every information about the specific instance of the problem
      simulator       % rbt v-rep
      qinit           % initial position 
      qdinit          % initial velocity
      time_sym_struct %time struct for simulation with fixed step
      fixed_step      % if is true i use ode4 (runge-kutta)
      options         % options for variable step 
      fitness         % fitness function handle
      fitness_result  % in this vector i save the value of the fitness function 
   end
       
    
   methods
       
       function obj = Instance(controller,simulator_type,qinit,qdinit,time_sym_struct,fixed_step,fitness,options)
           obj.controller = controller;
           if(getnameidx({'rbt','v-rep'} , simulator_type{1}) ~= 0 )
             obj.simulator = simulator_type;
           end
           obj.qinit = qinit;
           obj.qdinit= qdinit;
           obj.time_sym_struct = time_sym_struct;
           obj.fixed_step = fixed_step;
           obj.fitness = fitness;
           obj.options = options;
           
       end
       
       % this function has to give back something that let me compute the
       % fitness function for that sample
       function [t, q, qd]=run(obj,parameters)
        disp('im in run')   
            obj.controller.UpdateParameters(parameters)
            
            if(strcmp(obj.simulator,'rbt'))
                tic 
                [t, q, qd]=DynSim(obj.time_sym_struct,obj.controller,obj.qinit,obj.qdinit,obj.fixed_step);
                toc 
            end
       end
       
       function [mean_performances bestAction]=CMAES(obj,start_action,niter,explorationRate)
          %Parameter space
          NumParam = obj.controller.GetTotalParamNum();
          % start_value for action
          settings.action = start_action;
          settings.minAction = ones(1,NumParam).*obj.controller.alpha{1}.range(1,1);
          settings.maxAction = ones(1,NumParam).*obj.controller.alpha{1}.range(1,2);

          %CMA-ES settings
          settings.nIterations = niter;     
          settings.explorationRate = explorationRate; %[0, 1]
          settings.fnForwardModel = @(obj_,a_,ismean_)EvaluateCMAES(obj_,a_,ismean_);
          settings.plotState = 1;         %{0,1} plot offsprings yes no

          %search optimal parameters
          [mean_performances bestAction] = obj.LearnCMAES(settings);

          figure;
          plot(mean_performances);      
      end
       
       
       
       
       
       
   end
    
end