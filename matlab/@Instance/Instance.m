classdef  Instance
    
   properties
      controller  % structure that contains every information about the specific instance of the problem
      simulator   % rbt v-rep
      result      % vector that contains all the informations that is needed for the fitness function
   end
       
    
   methods
       
       function obj = Instance(controller,simulator_type)
           obj.controller = controller;
           if(getnameidx({'rbt','v-rep'} , type) ~= 0 )
             obj.simulator = simulator_type;
           end
       end
       
       
       function run(obj,parameters)
           
            obj.controller.SetParameter(parameters);
            
            if(strcmp(obj.simulator,'rbt'))
                tic
                options= odeset('MaxStep',0.001);   
                obj.controller.subchains.nofriction().fdyn(time_struct.tf,obj.controller,qinit,qdinit,options);
                toc 
            end
       end
       
       function [mean_performances bestAction]=CMAES(obj,min_action,max_action,niter,explorationRate)
          %Parameter space
          NumParam = obj.controller.GetTotalParamNum();
          settings.action = zeros(1,NumParam);
          settings.minAction = ones(1,NumParam).*min_action;
          settings.maxAction = ones(1,NumParam).*max_action;

          %CMA-ES settings
          settings.nIterations = niter;     
          settings.explorationRate = explorationRate; %[0, 1]
          settings.fnForwardModel = @(a_, flg_)EvaluateCMAES(a_,flg_, obj); %TO FIX
          settings.plotState = 1;         %{0,1} plot offsprings yes no

          %search optimal parameters
          [mean_performances bestAction] = learnCMAES(settings);

          figure;
          plot(mean_performances);      
      end
       
       
       
       
       
       
   end
    
end