classdef CrossEntropy < handle
   
   properties
      size_action;
      maxAction;
      minAction;
      lambda;
      particles;
      weigths;
      instance;
      fnForwardModel;
      constraints_active;
   end
   
   methods
      function obj = CrossEntropy(instance,size_action,maxAction,minAction,nIterations,explorationRate,fnForwardModel) 
         % dimension of the paarameter space
         obj.size_action = size_action;
         obj.maxAction = maxAction;
         obj.minAction = minAction;
         % number of particle
         obj.lambda =  round(4 + 3 * log(size_action));
         % gaussian mixture weight
         obj.weigths = (1/obj.lambda)*ones(1,obj.lambda);
         % actual particle used that rapresents the single gaussian of the
         % mixture
         obj.particles(obj.lambda) = Particle(size_action,maxAction,minAction,instance.penalty_handling.n_constraint,nIterations,explorationRate);
         %% i have to initialize the mean of each particle in some way
         % particle mean update
         % object that contains the actual forward model and the constraints function 
         obj.instance = instance;
         %forwardmodel
         obj.fnForwardModel = fnForwardModel;
         % flag that control the activation of the constraints
         obj.constraints_active = instance.penalty_handling.constraints;
      end
      
      
      
      function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = CrossEntropyOptimzation(obj)
         
         performances = zeros(obj.lambda, 1);
         offsprings = zeros(obj.lambda, obj.size_action);
         particleIndex = zeros(obj.lambda, 1);
         % in this way i associate forward model to the copy of instance in this class
         ForwardModel = @(obj,actionLearn_,curr_candidate_,isMean_)TransAction(obj,actionLearn_,curr_candidate_,isMean_);
         for k = 1:(nIterations - 1)
            if settings.plotState
               fprintf('iteration %d\n',k);
            end
            %% create offspring
            disp('create offsprings')
            for l = 1:obj.lambda 
               [candidate,mixtureIndex] = obj.Sample(k); 
               particleIndex(l) = mixtureIndex;
               offsprings(mixtureIndex, :) = candidate;
            end
            %% compute perfomance for each candidates
            for l = 1:obj.lambda 
               [performances(l) succeeded(policyId)] = ForwardModel(obj,offsprings(l, :),l, 0);
               %% added part to manage constraints
                if(obj.constraints_active)      
                    constraints = obj.penalty_handling.penalties(1,:)>0; % vector of index of the violated constrained
                    violated_constrained = find(constraints);
                    % the particle became active when no constraints is
                    % violated
                end 
               if settings.plotState
                   fprintf('Offspring %d : %e %d\n', l, performances(l), succeeded(policyId));
               end
            end
            
            
            
         end
      end
      
      
      function [candidate,mixtureIndex] = Sample(obj,index)
         % select one of the gaussian of the mixture 
         %mixtureIndex = 
         candidate = obj.particles(mixtureIndex).GetMean(index) + mvnrnd( zeros(1, n), obj.particles(mixtureIndex).GetCov(cov,index));
         % saturation
         candidate(1, candidate(1,:) > obj.maxAction) = obj.maxAction(candidate(1,:) > obj.maxAction);
         candidate(1, candidate(1,:) < obj.minAction) = obj.minAction(candidate(1,:) < obj.minAction);
      end
      % i use this function when im still in the condition where no
      % particle is active
      function UpdateParticleInactive(obj)
         
      end
      % i use this function when im still in the condition when mean of the
      % particle is in the feasible region
      function UpdateParticleActive(obj)
         
      end
      
      function [performance, succeeded, data2save ] = TransAction(obj,actionLearn,curr_candidate,isMean)
         if isfield(settings, 'activeIndices')
             if size(actionLearn,1) < 2
                 actionFull = settings.action;
                 actionFull(settings.activeIndices) = actionLearn;
             else
                 actionFull = settings.actionMultiple;
                 actionFull(settings.activeIndicesMultiple) = actionLearn;
             end
         else
             actionFull = actionLearn;
         end
         [performance, succeeded, data2save] = obj.fnForwardModel(obj.instance, actionFull, curr_candidate ,isMean);
      end
      
      
   end
   
end