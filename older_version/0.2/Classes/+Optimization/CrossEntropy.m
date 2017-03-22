%% TO DO:
% - add function for initializing the particle by covering all the space
%   possible
% - clean membership_weights at the end of each iteration

%% things already done so far:
% this class implment a mixture of gaussian Cross entropy method
% the single particle is rapresented as a 1+1 cmaes but they are used just
% as container of mean and the variance for each gaussian in the mixture



classdef CrossEntropy < handle
   
   properties
      activeIndices 
      plotState;
      size_action;
      maxAction;
      minAction;
      lambda;
      nIterations;
      particles;
      weigths;
      membership_weights;
      instance;
      fnForwardModel;
      constraints_active;
      gamma; % percentile value for the elites selection
      learning_rate;
      y_prc;
   end
   
   methods
      function obj = CrossEntropy(instance,plot_state,size_action,maxAction,minAction,nIterations,explorationRate,fnForwardModel) 
         obj.plotState = plot_state;
         % dimension of the parameter space
         obj.size_action = size_action;
         obj.maxAction = maxAction;
         obj.minAction = minAction;
         % number of particle
         obj.lambda =  round(4 + 3 * log(size_action));
         % number of iterations
         obj.nIterations = nIterations;
         % gaussian mixture weight
         obj.weigths = (1/obj.lambda)*ones(1,obj.lambda);
         % actual particle used that rapresents the single gaussian of the
         % mixture
         for i=1:obj.lambda
         obj.particles{i} = Optimization.ParticleExperimental(size_action,maxAction,minAction,instance.penalty_handling.n_constraint,nIterations,explorationRate);
         end
         %% i have to initialize the mean of each particle in some way
         % particle mean update
         obj.InitParticle()
         % object that contains the actual forward model and the constraints function 
         obj.instance = instance;
         %forwardmodel
         obj.fnForwardModel = fnForwardModel;
         % flag that control the activation of the constraints
         obj.constraints_active = instance.constraints;
         obj.gamma = -inf; 
         obj.y_prc =-inf;
         obj.learning_rate = 0.1;
      end
      
      function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = CrossEntropyOptimzation(obj)         
         performances = zeros(obj.lambda, 1);
         offsprings = zeros(obj.lambda, obj.size_action);
         particleIndex = zeros(obj.lambda, 1);
         % in this way i associate forward model to the copy of instance in this class
         ForwardModel = @(obj,actionLearn_,curr_candidate_,isMean_)TransAction(obj,actionLearn_,curr_candidate_,isMean_);
         for iter = 1:obj.nIterations
            if obj.plotState
               fprintf('iteration %d\n',iter);
            end
            %% create offspring
            %disp('create offsprings')
            for l = 1:(obj.lambda*2) 
               [candidate,mixtureIndex] = obj.Sample(iter); 
               particleIndex(l) = mixtureIndex;
               offsprings(l , :) = candidate;
            end
            %% compute perfomance for each candidates
            for l = 1:(obj.lambda*2) 
               [performances(l) succeeded] = ForwardModel(obj,offsprings(l, :),l, 0);
               %% added part to manage constraints
                if(obj.constraints_active)      
                    constraints = obj.instance.penalty_handling.penalties(1,:)>0; % vector of index of the violated constrained
                    violated_constrained = find(constraints);
                    % the particle became active when no constraints is
                    % violated
                end 
               if obj.plotState
                   fprintf('Offspring %d : %e %d\n', l, performances(l), succeeded);
               end
            end
            % sort sample 
            [sortPerf, sortInd] = sort(performances, 'ascend');
            % update percentile
            Y = prctile(sortPerf,50);
            if(Y>obj.y_prc)
                obj.y_prc = Y;
            end
            index = sortPerf>=obj.y_prc;
            elites = offsprings(sortInd(index),:);
            % update mixture of gaussian
            obj.MembershipAnalisys(elites,iter)
            for k=1:length(obj.particles)
                obj.UpdateWeightsCE(k);
                obj.UpdateMeanCE(k,elites,iter + 1 );
                obj.UpdateCovarianceCE(k,elites,iter + 1);
            end
            % plot particles on line
            if(true)
                for k=1:length(obj.particles)
                    obj.particles{k}.Plot(iter)
                end
            end
            %disp('iteration end')
            if(true)
                close all
            end
         end
      end
      function [candidate,mixtureIndex] = Sample(obj,index)
         % select one of the gaussian from the mixture 
         mixtureIndex = discretesample(obj.weigths,1);
         candidate = obj.particles{mixtureIndex}.GetMean(index) + mvnrnd( zeros(1, obj.size_action),obj.particles{mixtureIndex}.GetCov(index));
         % saturation
         candidate(1, candidate(1,:) > obj.maxAction) = obj.maxAction(candidate(1,:) > obj.maxAction);
         candidate(1, candidate(1,:) < obj.minAction) = obj.minAction(candidate(1,:) < obj.minAction);
      end
      % this function take the elites and compute the membership_weights
      % for all of them
      function MembershipAnalisys(obj,elites,iter)
          for i=1:size(elites,1)
              for j=1:length(obj.particles)
                  obj.membership_weights(i,j) = obj.weigths(1,j)*mvnpdf(elites(i,:),obj.particles{j}.GetMean(iter),obj.particles{j}.GetCov(iter));
              end
              % sum along column of the i-th rows
              normalization = sum(obj.membership_weights(i,:),2);
              obj.membership_weights(i,:) = obj.membership_weights(i,:)/normalization;
          end
      end
      % i update the particle k-th weight using the information from all the
      % particle
      function UpdateWeightsCE(obj,k)
          % number of elites
          N = size(obj.membership_weights,1);
          % sum along rows for the k-th column (N_k)
          norm_factor = sum(obj.membership_weights(:,k),1);
          obj.weigths(:,k) = obj.weigths(:,k) + obj.learning_rate*norm_factor/N; 
      end
      % i update the particle k-th mean using the information from all the
      % particle
      function UpdateMeanCE(obj,k,elites,iter)
          norm_factor = sum(obj.membership_weights(:,k),1);
          new_mean = zeros(size(elites(1,:)));
          for i = 1:size(elites,1)
              % read the column of the membership_weights and i sum up the
              % contributions of each elite to the k-th particle
              new_mean = new_mean + ( elites(i,:) - obj.particles{k}.GetMean(iter - 1) ) * obj.membership_weights(i,k);
          end
          %%DEBUG
          new_mean
          %%
          % normalization for N_k
          new_mean = obj.particles{k}.GetMean(iter - 1) + obj.learning_rate*(new_mean/norm_factor);
          obj.particles{k}.SetMean(new_mean,iter);
          %end
      end
      % i update the particle k-th covariance using the information from all the
      % particle
      function UpdateCovarianceCE(obj,k,elites,iter)
          norm_factor = sum(obj.membership_weights(:,k),1);
          new_C = zeros(size(obj.particles{1}.GetCov(iter-1)));
          for i = 1:size(elites,1)
              dist = (elites(i,:) - obj.particles{k}.GetMean(iter-1));
              new_C = new_C + obj.membership_weights(i,k)*(dist'*dist);
          end
          new_C = new_C / norm_factor;
          % with this i prevent the covariance to shrink to zero but to
          % keep a small value
          new_C = new_C + eye(length(elites(1,:)))*0.001;

          new_C = obj.particles{k}.GetCov(iter-1) + obj.learning_rate*new_C;
          %%DEBUG
          new_C
          %%
          obj.particles{k}.SetCov(new_C,iter);
      end
      
      %initialize particles
      % for this task im gonna use the hypercube latin sampling 
      function InitParticle(obj)
          s=lhsu(obj.minAction,obj.maxAction,length(obj.particles));
          for k=1:length(obj.particles)
              obj.particles{k}.SetMean(s(k,:),1);
          end
      end
      
      % i use this function when im in the condition that the mean of the
      % particle is in the feasible region
      function UpdateParticleActive(obj,k)
      end
      
      function [performance, succeeded, data2save ] = TransAction(obj,actionLearn,curr_candidate,isMean)
         if isfield(obj, 'activeIndices')
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
