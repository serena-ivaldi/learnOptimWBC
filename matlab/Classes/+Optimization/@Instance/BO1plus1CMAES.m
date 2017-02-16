%% TODO 
%% adding rule in pruneaction in particle manager
%% adding flag to impeed again to do deploy particle when this phase is closed
%% adding a global search for optimal solution during the optimization phase that if find a better particle respect of the current best i will add it (here i use eci as surrogate)
%% TOFIX 
%% way that i compute the remaining busget at the disposal of the deploy phase
%% side effect su emergency particle fenomeno mai osservato ?????

%% for now the concept of the algortihm is that i start with many particles and then i converge to a (1+1)CMA-ES enhanced with GP
%% 2 phase plus 1 (deploy ---> optimization and if deploy fails in the first attempts it trigger an emergency phase)
%% this object just receive from outside the results of the simulations (fitness and constraints)
function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = BO1plus1CMAES(obj,settings)
    %% global flags(for the method) 
    visualization = true;      % visualize intermediate result for debug
    local_boost_switch = true;      % with this variable i control if the boost is active or not 
    global_boost_switch = true;
    prune_switch =true;       % with this variable i activate or deactivate the prune move
    emergency_switch =true;    % with this i activate the emerggency behaviour when it is needed
    zooming_switch = true;     % with this switch i control the zooming for the gaussian process itself      
    
    
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
   %% this is a parameter of the problem (higher more exploration == more local minima founded) TODO move it out
   lambda = 3; 
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
   
   %% TODO set all of this variables as algorithm parameters
   % deploy parameters
   deploy_particle_budget = 0.9;   % this a percentage that defines the maximum amount of turn that i want to spend to fully fill the particles seats 
   
   % optimization parameters
   %% TOFIX maybe is better to change this parameter in order to take into account the real number of turn they need to wait 
   %% because the depends on the number of particles still active in this way this variables became indipendent repsect of the number of particles
    % this variable control after how many turn i run a local search during the optimization phase (is done once for each particles) 
    %(for turn here i mean number of turn per particles(ex: 3 particles 3 turns per particle ---> 3*3 = 9 turns))
   number_of_times_full_particles_sweep = 3;
   % local boost is going to be updated with the actual number of particle
   local_boost_event_trigger = lambda*number_of_times_full_particles_sweep;  
   % this variable control after how many turn i run a global search during
   % the optimization phase (here for turn i mean actual turn = one iteration of the code) 
   % this action has to be at a lower frequency than the other so 
   delay_turn = 10;
   % even this value is subject to update during the exec of the code
   % because depends on local_boost_event_trigger that is going to be
   % updated
   global_boost_event_trigger = local_boost_event_trigger + delay_turn; 
   % emergency parameters 
   emergency_event_trigger = 8;                % this variables trigger the emergency mode when at the beginning i fail to deploy the particles 
   locality_treshold = 200;                    % with this threshold i activate the GP optimization around the local optimal solution inside emergency mode
   emergency_boost_search_trigger = 10;        % number of turn to switch from exploration with particle to boost move (local or global) in emergency mode
  
   % fixed starting value for iterator / counter                      
   % emergency variables
   turn_of_emergency = 0;
   emergency_counter = 1;         % this iterator is used to trigger the emergency phase if we do not find any 
   emergency_iterator = 1;
   out_of_emergency = false;        % with this flag i signal that im out of mergency it means that in general looking for the free region was not easy so whe we explore is better to
                                    % search around the emergency_particle;      
   % deploy variables 
   deploy = true;
   alternating_counter = 0;         % this is a simple counter to alternate between a broad exploration and a small one when an emergency solution is found for deploy the particles                         
   turn_of_deploy = 0;
   total_iteration = nIterations; 
   
   % optimization variables
   local_boost_counter = 1;        % this is the counter that is used to activated a local GP boost exploitation for all the particles still active 
   global_boost_counter = 1;
   particle_iterator = 1;         % this iterator is used to iterate trough the particle during the optimization phase
   
   for ii = 1:nIterations  
       %% --------------------------------------------------------- Emergency Mode (im not capable of finding a free region with the normal mode)  
       if( emergency_counter > emergency_event_trigger && emergency_switch )
           %% EMERGENCY actions
           if( emergency_iterator == 1)
               if( ~isempty(PM.particles{1}))
                   if(abs(PM.particles{1}.GetBestPerfomance()) <= locality_treshold)
                       %% GP_local_search 
                       [x_candidate]=GPLocalExploration(PM,BO,PM.particles{1},true,zooming_switch);
                       %% TODO when i do local exlploration is better to update the particle instead oc building a new one
                   else
                       %% GP_global_search
                       x_candidate = GPGlobalExploration(BO,true,zooming_switch);
                   end
               else
                   %% GP_global_search
                   x_candidate = GPGlobalExploration(BO,true,zooming_switch);
               end
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
           % with this counter i count the number of turn that i spend for
           % emrgency
           turn_of_emergency = turn_of_emergency + 1;
           %% update section
           if(emergency_iterator == 1)
               % check if the global search has produced a better solution
               % than the particle or the particle vector is empty
               if(PM.active_particles == 0 || -emergency_perfomance > PM.particles{1}.GetBestPerfomance())
                   PM.AddParticleInPosition(x_candidate,-emergency_perfomance,1);
                   PM.particles{1}.DeactivateConstraints();
               end    
           else
               % i have to pass - emergency_perfomance because the
               % particle maximize and the constraints has to be minimized
               PM.UpdateParticle(1,violated_constrained,z,x_candidate, -(emergency_perfomance) );
           end
       % update the internal iterator
       emergency_iterator = emergency_iterator + 1;
       %% checks section
       if(emergency_iterator >= emergency_boost_search_trigger)
           emergency_iterator = 1;
       end
       %% if i reach the free region (- emergency perfomance > 0) i get out from the emergency mode
       if(emergency_perfomance <= 0)
           % i save the particle in mergency particle WITHOUT activating
           % the constraints managements inside the particle
           emergency_particle = PM.particles{1}.CopyParticle();
           % for the first particle i reactivate the constraints
           PM.particles{1}.ActivateConstraints(); 
           % i put emergency swith to false (i have found at least one free point im happy)
           emergency_switch = false;
           % i signal that im getting out of emergency
           out_of_emergency = true;
           % i update the total number of iteration to update the maximum
           % number of allowed 
           total_iteration = total_iteration - turn_of_emergency;
           % i reuse this particle for searching an optimal on the original objective function
           % when i get out from emergency i need to rescale the budget for
           % exploration by subtracting all the iteration spent for the
           % emergency phase in order to have keep the same %
       end
       %% --------------------------------------------------------- Normal Mode        
       else  
           %% action selector (for now we just saturate the particles list and then we exploit)
           if( PM.active_particles < lambda && turn_of_deploy < ceil(total_iteration * deploy_particle_budget) && deploy)
               deploy = true;
               % with this counter i keep track of the number of deploy turn already spent 
               turn_of_deploy = turn_of_deploy + 1;
           else
               % after this deploy is ended and i will never get back there
               deploy = false;
           end
           %% DEPLOY actions (global or local action with GP)
           if(deploy)       
               % select the new point
               disp('deploy particles phase'); % this phase once is completed we will never do that again
               if(out_of_emergency && alternating_counter < 2)            
                   if(alternating_counter == 0)
                        %% local action by using GP aorund the emergency particle
                        x_candidate = GPLocalExploration(PM,BO,emergency_particle,false,zooming_switch);
                        alternating_counter = 1;
                   elseif(alternating_counter == 1)
                        %% local action by evolving the emergency particle 
                        [x_candidate,z] = emergency_particle.Sample();
                        alternating_counter = 2;
                   end
               else
                   %% gobal action for deploy new particle far away (Default Move)
                   x_candidate = GPGlobalExploration(BO,false,zooming_switch);
                   % in this way in the next turn i will look for a
                   % solution using the local optimizer
                   alternating_counter = 0;
               end
               emergency_counter = emergency_counter + 1;
               % compute the model
           %% OPTIMIZATION actions (GP boost local or global or single particle spin)    
           else
               % if i get into exploitation once i switch off the emergency mode
               disp('optimization boost');
               emergency_switch = false;
               if(local_boost_switch && (local_boost_counter > local_boost_event_trigger) )
                   disp('local boost');
                   [x_candidate,z] = GPLocalExploitation(PM,BO,particle_iterator,zooming_switch);
               elseif(global_boost_switch && (global_boost_counter > global_boost_event_trigger))
                   disp('global boost');
                   [x_candidate,z] = GPGlobalExploitation(PM,BO,particle_iterator,zooming_switch);
               else
                   %% particle selector (for now we just iterate through the particle in order)
                   [x_candidate,z] = PM.Sample(particle_iterator);
               end
           end
           %% Execution 
           disp('evaluate offsprings')
           [performances_new succeeded(ii)] = fnForwardModel(obj,x_candidate,1, 1); % compute fitness 
           emergency_perfomance = ArtificialConstraints(obj.penalty_handling.penalties); % compute emergency perfomance
           y = obj.penalty_handling.penalties;
           %% here i compute an artificial function obtained by summing all the constraints violation and clamping to zero 
           %% all the constraints if only one constraints is not satisfied
           y(end + 1) = emergency_perfomance;
           y(end + 1) = performances_new;
           % constraints check
           constraints = obj.penalty_handling.feasibility_vec(1,:)==-1; % vector of index of the violated constrained
           violated_constrained = find(constraints);
           %% Update for DEPLOY
           if(deploy)
                % Update after running the sample during DEPLOY
                if(isempty(violated_constrained))
                    disp('added particle')
                    PM.AddParticle(x_candidate,performances_new) 
                end
                % Update emergency particle (i ahve to put here alternating_counter==2 because in this way i update the particle after sampling from it)
                if(alternating_counter == 2)
                    emergency_particle.Evolve(violated_constrained,z,x_candidate,-(emergency_perfomance) )
                    % check on emergency particle if it gets to small
                end
                %% here goes revive particle for the emergency particle
           else
           %% Update after running the sample during OPTIMIZATION
               str = sprintf('current particle is %d.',particle_iterator);
               disp(str);
               % evolve selected particle
               PM.UpdateParticle(particle_iterator,violated_constrained,z,x_candidate,performances_new)
               if(global_boost_counter <= global_boost_event_trigger)
                   % update particle iterator 
                   particle_iterator = particle_iterator + 1;  
               end
               if(particle_iterator > lambda)
                   % i have complete one sweep of all the particle, restart particle iterator 
                   particle_iterator = 1;
                   % at the end of each sweep i check if we need to prune some
                   % particle that has become redundant
                   if(prune_switch)
                        PM.PruneParticles();
                   end
                   if(local_boost_counter > local_boost_event_trigger)
                       % if im here it means that i have boosted all the
                       % particle so it time to restart the local_boost_counter and
                       % restore the former surrogate
                       % restore former surrogate function
                       %BO.SetSurrogate('ecv');
                       %restart_boost_counter
                       local_boost_counter = 1;
                   elseif(global_boost_counter <= global_boost_event_trigger)
                       % if im here it means that im still waiting to reach the
                       % next local boost move and im not doing any global boost at the time (i do not count it) 
                       local_boost_counter = local_boost_counter + 1; 
                   end
               end
               % i update the global boost only when im actually spinning
               % the particles ( i do not count the local boost to advance the global one)
               if(local_boost_counter <= local_boost_event_trigger && global_boost_counter <= global_boost_event_trigger)
                    % the global_boost_counter_are_actual_turn
                   global_boost_counter = global_boost_counter + 1;
               else
                   global_boost_counter = 1;
               end
               % update of the threshold for the local and global boost
               % actions
               local_boost_event_trigger = PM.active_particles *number_of_times_full_particles_sweep;
               global_boost_event_trigger = local_boost_event_trigger + delay_turns;
               %% revive particle goes here
           end
       end    
       %% plot
       if(visualization)
           BO.Plot(x_candidate);
           BO.PlotArtificial();
           %BO.PlotSurrogateByKind('pcs_constr');
           if(~deploy)    
               if(particle_iterator - 1 == 0)
                   PM.Plot(x_candidate,lambda,true,false);
               else
                   PM.Plot(x_candidate,particle_iterator  - 1,true,false);
               end
           else
               if(out_of_emergency)
                   figure
                   emergency_particle.Plot();
                   axis normal;
                   axis([minAction(1,1),maxAction(1,1),minAction(1,2),maxAction(1,2)])
               end
               PM.Plot(x_candidate,[],false,false);  
           end
       end
       %% update gaussian process (i keep updating the gaussian process even during the emergency phase)
       disp('update');
       BO.Update(x_candidate, y);
       %% collect data for the visualization (only if i have active particles (why?) )
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
       BO.PlotArtificial();
       BO.PlotSurrogateByKind('pcs_constr');
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


function [x_candidate]=GPLocalExploration(PM,BO,emergency_particle,EM_flag,zooming_switch)
    [mu,V_s,tlb,tub] = emergency_particle.GetRotTraslBound();
    transf = @(x_)emergency_particle.RotoTrasl(x_,mu,V_s);
    transf_for_AcqMax = @(x_)emergency_particle.RotoTraslWithoutSaturation(x_,mu,V_s);
    % insert zooming
    if(zooming_switch)
        % for the radius i get 
        BO.ZoomingIn(mu,tub);
    end
    if(EM_flag) 
         % emergency mode (here when im getting closer to the free
         %region i start to zoom using my gaussian process to be more
         %effective to find a free point)
         custom_function = @(x_)BO.pcs_constr(transf(x_));
         BO.SetSurrogate('custom',custom_function);
    else
         % (here because i came from an emergency mode it means that the
         % chances that i will find a new point away from the first free
         % point is low so i have to give a shot around the first point)
         mean = [];
         for iii=1:PM.active_particles
             mean = [mean;PM.particles{iii}.GetMean()];
         end
         custom_function = @(x_)BO.mcd_constr(transf(x_),mean);
         BO.SetSurrogate('custom',custom_function);
    end
    %% y_res used to debug
    [x_res,y_res] = BO.AcqMax(tlb,tub,transf_for_AcqMax);
    x_candidate = transf(x_res);
    %% TODEBUG
    PM.Plot([],1,false,true);
end

function y = ArtificialConstraints(penalties)
    % i check if there are violated constraints
    % in that case pull at all the non violated and sum up the violation
    % otherwise i sum all the constraints that are satisfied
    %% i check for all the constraints that are satisfied 
    index = penalties < 0; % in index: 0 = not satisfied      1 = satisfied
    %% because if the product is zero means that i have violation i sum only the violation 
    %% otherwise i sum all the satisfaction 
    if((prod(index)))
        y = sum(penalties);
    else
        penalties(index) = 0;
        y = sum(penalties);
    end
end

function [x_candidate]=GPGlobalExploration(BO,EM_flag,zooming_switch)
    %% remove zooming
    if(zooming_switch)
        BO.ZoomingOut();
    end
    if(EM_flag)
        % emergency mode (here i do not want to explore i want to
        % get straight to the free region as fast as possible so im gonna use
        % directly the probability of constraints violation surrogate even if 
        % im searching on a global scale)
        BO.SetSurrogate('pcs_constr');
    else 
        % ordinary mode (here i want to find the free region but 
        % i want mainly to explore the entire bounding box so i relay on a 
        % constraints variance surrogate)
        BO.SetSurrogate('ecv');
    end
    x_candidate = BO.AcqMax();
end

% in this function i try to accelerate the optimization path of the
% particle by giving an insight about where is located the local optimal in
% the area covered by the gaussian distribution of the particle in order to
% BOOST the search process
function [x_candidate,z]=GPLocalExploitation(PM,BO,particle_index,zooming_switch)
    [mu,V_s,tlb,tub] = PM.GetRotTraslFuncAndBound(particle_index);
    A  = PM.particles{particle_index}.GetCholCov();
    transf = @(x_)emergency_particle.RotoTrasl(x_,mu,V_s);
    transf_for_AcqMax = @(x_)emergency_particle.RotoTraslWithoutSaturation(x_,mu,V_s);
    % insert zooming
    if(zooming_switch)
        % for the radius i get 
        BO.ZoomingIn(mu,tub);
    end
    custom_function = @(x_,vararg)BO.eci(transf(x_),vararg);
    BO.SetSurrogate('custom',custom_function);
    x_res = BO.AcqMax(tlb,tub,transf_for_AcqMax);
    x_candidate = transf(x_res);
    % i have removed sigma from the equation to avoid explotion in the
    % covariance of the particle after the boost move
    %z=( A\(x_candidate - mu')' )/sigma;
    z =( A\(x_candidate - mu)' );
    z = z';
    %% TODEBUG i removed it because is better to actively zoom in and out when i need (lazy zooming)
%     % remove zooming
%     if(zooming_switch)
%         for i=1:length(BO.gp_s)
%              BO.gp_s{i}.DeactivateZooming()
%         end
%     end
    %% TODEBUG
    PM.Plot([],particle_index,false,true);
    
end


function [x_candidate]=GPGlobalExploitation(BO,zooming_switch)
%% remove zooming
    if(zooming_switch)
        BO.ZoomingOut();
    end
   % with this surrogate i should take into account which is the current
   % global maximum and see if there are other point in the space that
   % looks promising
    BO.SetSurrogate('eci');
    x_candidate = BO.AcqMax();
end
% this is a move that i perform only if i have only one active particle and
% the inaction plus the volume of the gaussian is getting to small to make
% it move. so i basically reinflate the gaussian in the hope that if there
% is something better around the particle im will find it
function ReviveParticle()
end
