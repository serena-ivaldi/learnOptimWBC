function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = BO1plus1CMAES(obj,settings)
    %% global flags(for the method) 
    visualization = true;    % visualize intermediate result for debug
    boost_switch = true;      % with this variable i control if the boost is active or not 
    prune_switch = true;      % with this variable i activate or deactivate the prune move
     emergency_switch = true;
    
    
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
   performances = zeros(1,nIterations);  % row vector
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
   % initialize the variables to track the best results
   max_perfomance = -inf;
   best_action = minAction;
   
   for jj = 1:length(init_y)
       if(init_y(jj)>max_perfomance)
           max_perfomance = init_y(jj);
           best_action = init_x(jj);
           BestActionPerEachGen(1,:) = best_action;
       end
   end

   %% optimization loop
   %% TODO set all boost variables as algorithm parameters
   boost_event_trigger = 6; % this variable determines after how many turn i activate the boost (for turn i mean number of update for the full particles stack) 
   boost_counter = 1;
   particle_iterator = 1;
   emergency_counter = 1;
   emergency_event_trigger = 5;
   emergency_global_search_trigger = 10; % number of turn to switch from exploration with particle to exploration with 
   emergency_iterator = 1;
   for ii = 1:nIterations  
       %% --------------------------------------------------------- Emergency Mode (im not capable of finding a free region with the normal mode)
       if(emergency_counter > emergency_event_trigger && emergency_switch)
           if( emergency_iterator == 1)
               %% global_search
               x_candidate = LookForFreeRegion(BO);
           else
               %% local_search (in emergency mode i have only one particle active
               [x_candidate,z] = PM.Sample(1);
           end
           %% check the candidate
           [performances_new succeeded(ii)] = fnForwardModel(obj,x_candidate,1, 1); % compute fitness 
           emergency_perfomance = ArtificialConstraints(obj.penalty_handling.penalties); % compute emergency perfomance
           y = obj.penalty_handling.penalties;
           y(end + 1) = emergency_perfomance;
           y(end + 1) = performances_new;
           %% update section
           if(emergency_iterator == 1)
               % check if the global search has produced a better solution
               % than the particle or the particle vector is empty
               if(PM.active_particles == 0 || -emergency_perfomance > PM.particles{1}.GetBestPerfomance())
                   PM.AddParticleInPosition(x_candidate,-emergency_perfomance,1);
                   PM.particles{1}.DeactivateConstraints();
               end
               emergency_iterator = emergency_iterator + 1;
           else
               % i have to pass - emergency_perfomance because the
               % particle maximize and the constraints has to be minimized
               PM.UpdateParticle(1,violated_constrained,z,x_candidate, -(emergency_perfomance) );
               emergency_iterator = emergency_iterator + 1;
           end
       % update the internal iterator    
       if(emergency_iterator >= emergency_global_search_trigger)
           emergency_iterator = 1;
       end    
       %% if i reach the free region (- emergency perfomance > 0) i get out from the emergency mode
       if(emergency_perfomance < 0)
           emergency_switch = false;
           PM.particles{1}.ActivateConstraints(); % i reuse this particle for searching an optimal on the original objective function
       end
       %% --------------------------------------------------------- Normal Mode        
       else  
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
               emergency_counter = emergency_counter + 1;
               % compute the model
           else
               %% exploitation (local action)
               % if i get into exploitation once i switch off the emergency mode
               emergency_switch = false;
               if(boost_switch && (boost_counter > boost_event_trigger) )
                   disp('boost to debug');
                   [x_candidate,z] = Boost(PM,BO,particle_iterator);
               else   
                   %% particle selector (for now we just iterate through the particle in order)
                   [x_candidate,z] = PM.Sample(particle_iterator);
               end
           end
       
           %% execution 
           disp('evaluate offsprings')
           [performances_new succeeded(ii)] = fnForwardModel(obj,x_candidate,1, 1); % compute fitness 
           y = obj.penalty_handling.penalties;
           %% here i compute an artificial function obtained by summing all the constraints violation and clamping to zero 
           %% all the constraints that are satisfied
           %y(end + 1)= obj.penalty_handling.feasibility;
           y(end + 1) = ArtificialConstraints(obj.penalty_handling.penalties);
           y(end + 1) = performances_new;
           % constraints check
           constraints = obj.penalty_handling.feasibility_vec(1,:)==-1; % vector of index of the violated constrained
           violated_constrained = find(constraints);

           if(exploration)
                %% add a new particle (if im exploring and the point satisfy all the constraints)
                if(isempty(violated_constrained))
                    disp('added particle')
                    PM.AddParticle(x_candidate,performances_new) 
                end
           else
           %% update particle (if im exploiting)
               str = sprintf('current particle is %d.',particle_iterator);
               disp(str);
               % evolve selected particle
               PM.UpdateParticle(particle_iterator,violated_constrained,z,x_candidate,performances_new)
               % update particle iterator 
               particle_iterator = particle_iterator + 1;  
               if(particle_iterator > lambda)
                   % i have complete one sweep of all the particle, restart particle iterator 
                   particle_iterator = 1;
                   % at the end of each sweep i check if we need to prune some
                   % particle that has become redundant
                   if(prune_switch)
                        PM.PruneParticles();
                   end
                   if(boost_counter > boost_event_trigger)
                       % if im here it means that i have boosted all the
                       % particle so it time to restart the boost_counter and
                       % restore the former surrogate
                       % restore former surrogate function
                       BO.SetSurrogate('ecv');
                       %restart_boost_counter
                       boost_counter = 1;
                   else
                       % if im here it means that im still waiting to reach the
                       % next boost move
                       boost_counter = boost_counter + 1;
                   end
               end
           end
       end    
       %% plot
       if(visualization)
           BO.Plot(x_candidate);
           BO.PlotArtificial();
           if(~exploration)    
               if(particle_iterator - 1 == 0)
                   PM.Plot(x_candidate,lambda,true,false);
               else
                   PM.Plot(x_candidate,particle_iterator  - 1,true,false);
               end
           else
               PM.Plot(x_candidate,[],false,false);
           end
       end
       %% update gaussian process (i keep updating the gaussian process during the )
       disp('update');
       BO.Update(x_candidate, y)
       
       %% collect data for the visualization
       if(PM.active_particles > 0)
           for kk = 1:PM.active_particles
               if(PM.particles{kk}.GetBestPerfomance > max_perfomance)
                   % collectind data for the visualization
                   costs(ii) = -PM.particles{kk}.GetBestPerfomance();
                   bestAction.hist(ii).performance = PM.particles{kk}.GetBestPerfomance();
                   bestAction.hist(ii).parameters = PM.particles{kk}.GetMean();
                   % update max
                   max_perfomance = PM.particles{kk}.GetBestPerfomance();
                   best_action = PM.particles{kk}.GetMean();
                   BestActionPerEachGen(1,:) = PM.particles{kk}.GetMean();

               else
                   costs(ii) = -max_perfomance;
                   bestAction.hist(ii).performance = max_perfomance;
                   bestAction.hist(ii).parameters = best_action;
                   BestActionPerEachGen(ii,:) = best_action;
               end
           end
       else
           costs(ii) = -max_perfomance;
           bestAction.hist(ii).performance = max_perfomance;
           bestAction.hist(ii).parameters = best_action;
           BestActionPerEachGen(ii,:) = best_action;
       end
       
       %% plot
       if(visualization)
           pause(0.05)
           close all;
       end
   end 
   
   %% plot
   if(visualization)
       BO.Plot(x_candidate);
       PM.Plot([],[],false,false);
       pause();
       close all;
   end
   
   %% store data for visualization of the stats
   G_data2save.Particles = {PM.old_particles,PM.particles};
   G_data2save.A = [];
   G_data2save.C = [];
   G_data2save.performance = [0];

   policies = [];

   bestAction.parameters = bestAction.hist(end).parameters;
   bestAction.performance = -costs(:,end);
   bestAction.listperformance = -costs;
   performances =  -costs;
      
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
    %init_x = [ 1 1;9 1;1 9;9 9;5 5];
    % Evaluate target function at all initialization
    % points (random + explore)
    for i=1:number_init_points
        % questa cosa va modificata perche la assegnazione va fatta dentro la funzione
       [init_performances] = fnForwardModel(obj,init_x(i,:),1, 1); % compute fitness  
       y = obj.penalty_handling.penalties';
       %y(end + 1)= obj.penalty_handling.feasibility;
       y(end + 1) = ArtificialConstraints(obj.penalty_handling.penalties);
       y(end + 1) = init_performances;
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


function [x_candidate,z]=Boost(PM,BO,particle_index)

    %[mu,V_s,tlb,tup] = PM.particles{particle_index}.GetRotTraslBound();
    [transf,tlb,tub] = PM.GetRotTraslFuncAndBound(particle_index);
    %sigma = PM.particles{particle_index}.GetSigma();
    A  = PM.particles{particle_index}.GetCholCov();
    mu = PM.particles{particle_index}.GetMean();
    %transf = @(x_)PM.RotoTrasl(x_,mu,V_s);
    custom_function = @(x_,xi_)BO.eci(transf(x_), xi_);
    BO.SetSurrogate('custom',custom_function);
    x_res = BO.AcqMax(tlb,tub);
    x_candidate = transf(x_res);
    % i have removed sigma from the equation to avoid explotion in the
    % covariance of the particle after the boost move
    %z=( A\(x_candidate - mu')' )/sigma;
    z =( A\(x_candidate - mu)' );
    z = z';
    
    %% TODEBUG
    PM.Plot([],particle_index,false,true);
    
end


function [x_res] = LookForFreeRegion(BO)
    current_kind = BO.kind;
    BO.SetSurrogate('ucb_constr');
    x_res = BO.AcqMax();
    BO.SetSurrogate(current_kind);
end

function y = ArtificialConstraints(penalties)
    % i all the constraints that are satisfied are set to zero 
    penalties(penalties<0) = 0;
    y = sum(penalties);
end
