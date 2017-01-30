function [mean_performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = BO1plus1CMAES(obj,settings)

    %% Initialization
    nIterations = settings.nIterations;

    if isfield(settings, 'activeIndices')
        action = settings.action(settings.activeIndices);
        minAction = settings.minAction(settings.activeIndices);
        maxAction = settings.maxAction(settings.activeIndices);
    else
        action = settings.action;
        minAction = settings.minAction;
        maxAction = settings.maxAction;
    end

    n = size(minAction,2);
    lambda = round(4 + 3 * log(n));
    n_constraints = obj.penalty_handling.n_constraint;

    fnForwardModel = @(obj_,actionLearn_,curr_candidate_,isMean_)TransAction(obj_,actionLearn_,curr_candidate_,isMean_, settings);
    
    %% TODO pass from outside the GP_lib parameter to specify which gaussian process library i wanna use
    BO = Optimization.BayesOpt(minAction, maxAction, n,n_constraints,'GP_stuff');
    PM = Optimization.ParticleManager(lambda,n,n_constraints,maxAction,minAction,nIterations,settings.explorationRate);
    
    %% TODO for metrics i need to introduce the same structure fo the other method
    mean_performances = zeros(1,nIterations);  % row vector
    costs = zeros(1,nIterations);      
    succeeded = zeros(1,nIterations);
    %% TODO properly use G_data2save and policies
    G_data2save = [];
    policies = [];
    BestActionPerEachGen = ones(nIterations,n);
    %% initilization
    %% TODO put number_init_points in the initial value for this method
    number_init_points = 5;
    [init_x,init_y]=InitialSample(obj,fnForwardModel,minAction,maxAction,number_init_points);
    BO.Init(init_x,init_y)
  
    
   %% optimization loop
   particle_iterator = 1;
   for ii = 1:nIterations
       
       %% action selector (for now we just saturate the particles list and then we exploit)
       if(PM.active_particles < lambda)
           exploration = true;
       else
           exploration = false;
       end
       
       if(exploration)
           %% exploration (global action)
           % select the new point
           disp('optimization surrogate');
           x_candidate = BO.AcqMax();
           % compute the model
       else
           %% exploitation (local action)
           %% particle selector (for now we just iterate through the particle in order)
           [x_candidate,z] = PM.Sample(particle_iterator);
           % sample from that particle
       end
       %% execution 
       disp('evaluate offsprings')
       [performances_new succeeded(ii)] = fnForwardModel(obj,x_candidate,1, 1); % compute fitness 
       y = obj.penalty_handling.penalties;
       y(end + 1)= obj.penalty_handling.feasibility;
       y(end + 1) = performances_new;
       % constraints check
       constraints = obj.penalty_handling.feasibility_vec(1,:)==-1; % vector of index of the violated constrained
       violated_constrained = find(constraints);
       
       if(exploration)
            %% add a new particle (if im exploring and the point satisfy all the constraints)
            if(isempty(violated_constrained))
                PM.AddParticle(x_candidate,performances_new) 
            end
       else
       %% update particle (if im exploiting)
           % evolve selected particle
           PM.UpdateParticle(particle_iterator,violated_constrained,z,x_candidate,performances_new)
           % prune colliding particles
           PM.PruneParticles();  %%  TODO to code!
           % update particle iterator 
           particle_iterator = particle_iterator + 1;  
           if(particle_iterator > lambda)
               particle_iterator = 1;
           end
       end
       %% plot(TODEBUG)
       BO.Plot(x_candidate);
       PM.Plot();
       %% update gaussian process (i keep updating the gaussian process during the )
       disp('update');
       BO.Update(x_candidate, y)
       
       % Keep track of information about iteration 
%        self.i += 1
% 
%        self.res['max'] = {'max_val': self.Y.max(),
%                            'max_params': dict(zip(self.keys,
%                                                   self.X[self.Y.argmax()]))
%                            }
%        self.res['all']['values'].append(self.Y[-1])
%        self.res['all']['params'].append(dict(zip(self.keys, self.X[-1]))) 
       mean_perfomances(ii) = BO.y_max;
       if(~isempty(BO.x_max))
            BestActionPerEachGen(ii,:) = BO.x_max;
       end
       %% TODEBUG
       pause
   end 
   if(~isempty(BO.x_max))
       bestAction.parameters  = BO.x_max;
   else
       bestAction.parameters = ones(1,n);
   end
          
   bestAction.performance = BO.y_max;
  
  
   
   
end

% with this function we generate the set of points to initiliaze the search
function [init_x,init_y]=InitialSample(obj,fnForwardModel,lb,ub,number_init_points)

    %  Initialization method to kick start the optimization process. It is a
    %  combination of points passed by the user, and randomly sampled ones.
    % 
    %  param init_points: Number of random points to probe.
    % Create an array with parameters bounds
    
    %% TODO check on it
    % Generate random points (l is a matrix each row is a radom point)
    %r = a + (b-a).*rand(100,1);
    init_x = repmat(lb,number_init_points,1) + repmat(ub-lb,number_init_points,1).*rand(number_init_points,length(lb));
    
    %% TODEBUG provisory change (ho sovrascritto init_x )for confrontation with demobayesopt.m
    init_x = [ 1 1;9 1;1 9;9 9;5 5];
    % Evaluate target function at all initialization
    % points (random + explore)
    for i=1:number_init_points
        % questa cosa va modificata perche la assegnazione va fatta dentro la funzione
       [performances] = fnForwardModel(obj,init_x(i,:),1, 1); % compute fitness  
       y = obj.penalty_handling.penalties';
       y(end + 1) = obj.penalty_handling.feasibility;
       y(end + 1) = performances;
       init_y(i,:) = y;
        %if self.verbose
            %self.plog.print_step(x, y_init[-1])
        %end
    end
end


function [performance, succeeded, data2save ] = TransAction(obj_,actionLearn, curr_candidate,isMean, settings)
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
        [performance, succeeded, data2save] = settings.fnForwardModel(obj_, actionFull, curr_candidate ,isMean);
end
