%% IDEAS
%%  it is possible to add a fourth move where i use a great number of point sampled inside the gaussian of the particle but the results are comuted from the surrogate function
%% to accelerate the evolution of the covariance (we should not use point that satisfy the constraints and improve the fitness)
%% TODO
%% TOFIX 
%% way that i compute the remaining deploy budget after mergency phase 


%% for now the concept of the algortihm is that i start with many particles and then i converge to a (1+1)CMA-ES enhanced with GP
%% 2 phase plus 1 (deploy ---> optimization and if deploy fails (do not complete the deploy) in the first attempts it trigger an emergency phase to accelerate the process)
%% this object just receive from outside the results of the simulations (fitness and constraints)
function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = BO1plus1CMAES(obj,settings)
    %% global flags(for the method) 
    debug = true;
    visualization = false;           % visualize intermediate result for debug
    visualization_for_paper = false;  % visualize stuff for the paper
    local_boost_switch = true;       % with this variable i control if the boost is active or not 
    global_boost_switch = true;
    prune_switch =true;       % with this variable i activate or deactivate the prune move
    emergency_switch =true;    % with this i activate the emergency behaviour when it is needed
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
   %% TODO this is a parameter of the problem (higher more exploration == more local minima founded) TODO move it out
   lambda = 3; 
   
   n_constraints = obj.penalty_handling.n_constraint;
   fnForwardModel = @(obj_,actionLearn_,curr_candidate_,isMean_)TransAction(obj_,actionLearn_,curr_candidate_,isMean_, settings);
    
   %% TODO pass from outside the GP_lib parameter to specify which gaussian process library i wanna use
   BO = Optimization.BayesOpt(minAction, maxAction, n,n_constraints,'GP_stuff');
   %% TODO set this variable of BO outside BO
   % self.kappa =0.01;
   % self.xi = 0;
   PM = Optimization.ParticleManager(lambda,n,n_constraints,maxAction,minAction,nIterations,settings.explorationRate);
   %% TODO define this varible outside PM
   %obj.inaction_limit = 20;
   %obj.local_GP_reboot_threshold = 2; % (is not gonna harm the computation a small value)
    
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
   local_boost_event_trigger = number_of_times_full_particles_sweep;  
   % this variable control after how many turn i run a global search during
   % the optimization phase (here for turn i mean actual turn = one iteration of the code) 
   % this action has to be at a lower frequency than the other so 
   delay_turn = 15;
   % even this value is subject to update during the exec of the code
   % because depends on local_boost_event_trigger that is going to be
   % updated
   global_boost_event_trigger = local_boost_event_trigger*lambda + delay_turn; 
   % emergency parameters 
   emergency_event_trigger = 4;                % this variables trigger the emergency mode when at the beginning i fail to deploy the particles 
   locality_treshold = 200;                    % with this threshold i activate the GP optimization around the local optimal solution inside emergency mode
   emergency_boost_search_trigger = 10;        % number of turn to switch from exploration with particle to boost move (local or global) in emergency mode
  
   % fixed starting value for iterator / counter                      
   % emergency variables
   if(visualization_for_paper)
       emergency_particle_old = [];
   end
   emergency_particle = [];
   current_real_fitness_for_emergency_particle = 0; 
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
       fprintf('iteration %d\n',ii);
       %% --------------------------------------------------------- Emergency Mode (im not capable of finding a free region with the normal mode)  
       if( emergency_counter > emergency_event_trigger && emergency_switch )
           if(debug)
               disp('emergency');
           end
           %% EMERGENCY actions
           if( emergency_iterator == 1)
               if( ~isempty(emergency_particle))
                   if(abs(emergency_particle.GetBestPerfomance()) <= locality_treshold)
                       %% GP_local_search 
                       if(debug)
                           disp('emergency GP local search')
                       end
                       if(visualization_for_paper)
                           act = 'local';
                       end
                       [x_candidate]=GPLocalExploration(PM,BO,emergency_particle,true,zooming_switch,visualization);
                       %% TODO when i do local exlploration is better to update the particle instead oc building a new one
                   else
                       %% GP_global_search
                       if(debug)
                           disp('emergency GP global search')
                       end
                       if(visualization_for_paper)
                           act = 'global';
                       end
                       x_candidate = GPGlobalExploration(BO,true,zooming_switch);
                   end
               else
                   %% GP_global_search
                   if(debug)
                       disp('emergency GP global search')
                   end
                   if(visualization_for_paper)
                       act = 'global';
                   end
                   x_candidate = GPGlobalExploration(BO,true,zooming_switch);
               end
           else
               %% local_search (in emergency mode i have only one particle active
               if(debug)
                   disp('emergency particle sample')
               end
               if(visualization_for_paper)
                   act = 'sample';
               end
               [x_candidate,z] = emergency_particle.Sample();
           end
           %% check the candidate
           [performances_new succeeded(ii)] = fnForwardModel(obj,x_candidate,1, 1); % compute fitness 
           constraints_violation_cost = ArtificialConstraints(obj.penalty_handling.penalties); % compute emergency perfomance
           y = obj.penalty_handling.penalties;
           y(end + 1) = constraints_violation_cost;
           y(end + 1) = performances_new;
           % constraints check (i do not need this part here because im optimizing only looking at the constraints fitness)
           %constraints = obj.penalty_handling.feasibility_vec(1,:)==-1; % vector of index of the violated constrained
           %violated_constrained = find(constraints);
           % with this counter i count the number of turn that i spend for
           % emergency
           turn_of_emergency = turn_of_emergency + 1;
           %% update section
           if(emergency_iterator == 1)
               % check if the GP global search or the GP local search has produced a better solution
               % or the emergency particle is empty. in both case i replace
               % the particle 
               if(isempty(emergency_particle) || -constraints_violation_cost > emergency_particle.GetBestPerfomance())
                   if(visualization_for_paper)
                       emergency_particle_old = emergency_particle;
                   end
                   % create new emergency_particle
                   emergency_particle = Optimization.Particle(PM.size_action,PM.maxAction,PM.minAction,PM.n_constraints,PM.nIterations,...
                                                             PM.explorationRate,x_candidate,-constraints_violation_cost,[1 0 0]);
                   emergency_particle.DeactivateConstraints();
                   % update current real fitness value
                   current_real_fitness_for_emergency_particle = performances_new;
               end    
           else
               % here i update the
               % current_real_fitness_for_emergency_particle only if the
               % particle move and it happens only when particle fitness
               % get better (in this case the fitness im talking about is the constraint violation fitness)
               if(-constraints_violation_cost > emergency_particle.GetBestPerfomance())
                   current_real_fitness_for_emergency_particle = performances_new;
               end
               % i have to pass - constraints_violation_cost because the
               % particle maximize and the constraints has to be minimized
               emergency_particle.Evolve(violated_constrained,z,x_candidate,-(constraints_violation_cost));
           end
       % update the internal iterator
       emergency_iterator = emergency_iterator + 1;
       %% checks section
       if(emergency_iterator >= emergency_boost_search_trigger)
           emergency_iterator = 1;
       end
       %% if i reach the free region (- emergency perfomance > 0) i get out from the emergency mode
       if(constraints_violation_cost <= 0)
           if(debug)
               disp('out of emergency');
           end
           % i create from emergency particle the first PM particle that i
           % will use in the optimization process
           % PM.AddParticleInPosition(emergency_particle.GetMean(),current_real_fitness_for_emergency_particle,1,true);
           %% it can happen that a new particles are added before i get into the emergency so when i get out of emergency i 
           %% i have to add the new particle 
           PM.AddParticle(emergency_particle.GetMean(),current_real_fitness_for_emergency_particle,true)
           % i put emergency swith to false (i have found at least one free point, im happy)
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
               if(debug)
                   disp('deploy'); % this phase once is completed we will never do that again
               end
               if(out_of_emergency && alternating_counter < 2)            
                   if(alternating_counter == 0)
                        %% local action by using GP aorund the emergency particle
                        if(debug)
                            disp('deploy GP local exploration')
                        end
                        if(visualization_for_paper)
                            act = 'local';
                        end
                        x_candidate = GPLocalExploration(PM,BO,emergency_particle,false,zooming_switch,visualization);
                        alternating_counter = 1;
                   elseif(alternating_counter == 1)
                        %% local action by evolving the emergency particle
                        if(debug)
                           disp('deploy sample (emergency) particle')
                        end
                        if(visualization_for_paper)
                            act = 'sample';
                        end
                        [x_candidate,z] = emergency_particle.Sample();
                        alternating_counter = 2;
                   end
               else
                   %% global action for deploy new particle far away (Default Move)
                   if(debug)
                       disp('deploy GP global search')
                   end
                   if(visualization_for_paper)
                       act = 'global';
                   end
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
               if(debug)
                   disp('optimization');
               end
               emergency_switch = false;
               if(local_boost_switch && (local_boost_counter > local_boost_event_trigger) )
                   if(debug)
                       disp('optimization local boost');
                   end
                   if(visualization_for_paper)
                       act = 'local';
                   end
                   [x_candidate,z] = GPLocalExploitation(PM,BO,particle_iterator,zooming_switch,visualization);
               elseif(global_boost_switch && (global_boost_counter > global_boost_event_trigger))
                   if(debug)
                       disp('optimization global boost');
                   end
                   if(visualization_for_paper)
                        act = 'global';
                    end
                   [x_candidate] = GPGlobalExploitation(PM,BO,zooming_switch);
               else
                   if(debug)
                       disp('sample from particle');
                       str = sprintf('optimization: current particle is %d.',particle_iterator);
                       disp(str);
                   end
                   %% particle selector (for now we just iterate through the particle in order)
                   if(visualization_for_paper)
                        act = 'sample';
                   end
                   [x_candidate,z] = PM.Sample(particle_iterator);
               end
           end
           %% Execution 
           if(debug)
                disp('execution (deploy and optimization)')
           end
           [performances_new succeeded(ii)] = fnForwardModel(obj,x_candidate,1, 1); % compute fitness 
           constraints_violation_cost = ArtificialConstraints(obj.penalty_handling.penalties); % compute emergency perfomance
           y = obj.penalty_handling.penalties;
           %% here i compute an artificial function obtained by summing all the constraints violation and clamping to zero 
           %% all the constraints if only one constraints is not satisfied
           y(end + 1) = constraints_violation_cost;
           y(end + 1) = performances_new;
           % constraints check
           constraints = obj.penalty_handling.feasibility_vec(1,:)==-1; % vector of index of the violated constrained
           violated_constrained = find(constraints);
           %% Update for DEPLOY
           if(deploy)
                % Update after running the sample during DEPLOY
                if(isempty(violated_constrained))
                    if(debug)
                        disp('deploy add particle');
                    end
                    PM.AddParticle(x_candidate,performances_new,true); 
                end
                % Update emergency particle (i ahve to put here alternating_counter==2 because in this way i update the particle after sampling from it)
                if(alternating_counter == 2)
                    emergency_particle.Evolve(violated_constrained,z,x_candidate,-(constraints_violation_cost) );
                    % check on emergency particle if it gets to small
                end
                %% TODO here goes revive particle for the emergency particle
           else
           %% Update after running the sample during OPTIMIZATION
               if(global_boost_counter <= global_boost_event_trigger)
                   % evolve selected particle
                   PM.UpdateParticle(particle_iterator,violated_constrained,z,x_candidate,performances_new,true);
                   if(local_boost_counter > local_boost_event_trigger)
                       % if i get there it means that im doing a sweep with the
                       % local GP boost so i have to update the internal
                       % counter of the current particle if the local boost was
                       % ineffective
                       PM.GetParticle(particle_iterator).UpdateCounterLocalGPBoost();
                   end
                   % update particle iterator 
                   particle_iterator = particle_iterator + 1; 
               else
                   %% TODO introduce best perfomance in PM (and only if the current point is better than best perfomance i create i new particle)
                   %% and change add.particle in orded to allow a flexible lambda value (i can have more particle than the number fixed at the beginning)
                   if(isempty(violated_constrained))
                       if(performances_new > PM.global_maximum_among_particles.cur_max)
                           if(debug)
                               disp('optimization add particle');
                           end
                           PM.AddParticle(x_candidate,performances_new,true);
                       end
                   end
               end
               
               % if have completed one sweep of all the particles, i will restart
               % particle iterator (this condition should protect me in case im doing a global )
               if(particle_iterator > PM.active_particles)
                   particle_iterator = 1;
                   % at the end of each sweep i check if we need to prune some
                   % particle that has become redundant (i do the check to remove the particle only if i have more than one particle)
                   if(prune_switch)
                        str = PM.PruneParticles();
                        if(debug)
                            if(~isempty(str))
                                disp(str);
                            end
                        end
                   end
                   if(local_boost_counter > local_boost_event_trigger)
                       % if im here it means that i have boosted all the
                       % particle so it time to restart the local_boost_counter and
                       % restore the former surrogate
                       % restore former surrogate function
                       %BO.SetSurrogate('ecv');
                       %restart_boost_counter
                       local_boost_counter = 1;
                   else
                       % if im here it means that im still waiting to reach the
                       % next local boost move and im not doing any global boost at the time (i do not count it) 
                       local_boost_counter = local_boost_counter + 1; 
                   end
               end
               % i update the global boost only when im actually spinning
               % the particles ( i do not count the local boost to advance the global one)
               %% with the update in this point i loose the last update of the last particle (i lose one turn) but i regain one from the last particle so it should be square
               if(local_boost_counter <= local_boost_event_trigger ) 
                   if(global_boost_counter <= global_boost_event_trigger)
                        % the global_boost_counter_are_actual_turn
                       global_boost_counter = global_boost_counter + 1;
                   else
                       global_boost_counter = 1;
                   end
               end
               %% TODO revive particle goes here
               if(PM.active_particles == 1)
                   str = ReviveParticle(PM);
                   if(debug)
                       if(~isempty(str))
                           disp(str);
                       end
                   end
               end
           end
       end    
       %% plot for debug
       if(visualization)
           BO.Plot(x_candidate);
           BO.PlotArtificial();
           %BO.PlotSurrogateByKind('pcs_constr');
           if(~deploy)    
               if(particle_iterator - 1 == 0)
                   PM.Plot(x_candidate,lambda,true,false);
               else
                   PM.Plot(x_candidate,particle_iterator - 1,true,false);
               end
           else    
               if(~isempty(emergency_particle))
                   figure
                   emergency_particle.Plot();
                   axis normal;
                   axis([minAction(1,1),maxAction(1,1),minAction(1,2),maxAction(1,2)])
               end
               PM.Plot(x_candidate,[],false,false);  
           end
       end
       %% this portion of code is devoted to the visualization of the images of the paper
       if(visualization_for_paper)
            corrected_particle_iterator = particle_iterator - 1;
            if(corrected_particle_iterator==0)
                corrected_particle_iterator = PM.active_particles;
            end
            state = StateMachine(deploy,emergency_switch,emergency_counter,emergency_event_trigger);
            success = MoveSuccessCheck(act,state,constraints_violation_cost,performances_new,emergency_particle,corrected_particle_iterator,PM);
            PaperPlot(state,act,emergency_particle,emergency_particle_old,PM,BO,x_candidate,corrected_particle_iterator);
       end
       %% update gaussian process (i keep updating the gaussian process even during the emergency phase)
       if(debug)
           disp('BO update');
       end
       % with this variable i state that i will update the hyperparameter and the covariance matrix only when i call it explicitly
       Train = false; 
       BO.Update(x_candidate, y,Train);
       %% collect data for the visualization (only if i have active particles (why?) )
       if(PM.active_particles > 0)
           if(PM.global_maximum_among_particles.cur_max ~= -inf)
               costs(ii) = -PM.global_maximum_among_particles.cur_max;
               bestAction.hist(ii).performance = PM.global_maximum_among_particles.cur_max;
           else
               costs(ii) = 0;
               bestAction.hist(ii).performance = 0;
           end
           bestAction.hist(ii).parameters = PM.global_maximum_among_particles.cur_best_action;
           BestActionPerEachGen(ii,:) = PM.global_maximum_among_particles.cur_best_action; 
       else
           dummyaction = zeros(1,n);
           costs(ii) = 0;
           bestAction.hist(ii).performance = 0;
           bestAction.hist(ii).parameters = dummyaction;
           BestActionPerEachGen(ii,:) = dummyaction;
       end
       
       %% plot
       if(visualization || visualization_for_paper)
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
   G_data2save.PM = PM;
   G_data2save.BO =BO;

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
    
    % Generate random points (l is a matrix each row is a radom point)
    %r = a + (b-a).*rand(100,1);
    init_x = repmat(lb,number_init_points,1) + repmat(ub-lb,number_init_points,1).*rand(number_init_points,length(lb));
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


function [x_candidate]=GPLocalExploration(PM,BO,emergency_particle,EM_flag,zooming_switch,visualization_switch)
    [mu,V_s,tlb,tub] = emergency_particle.GetRotTraslBound();
    sigma = emergency_particle.GetSigma();
    sigma_mult = emergency_particle.GetSigmaMultiplier();
    transf = @(x_)emergency_particle.RotoTrasl(x_,mu,V_s);
    transf_for_AcqMax = @(x_)emergency_particle.RotoTraslWithoutSaturation(x_,mu,V_s);
    % insert zooming
    if(zooming_switch)
        %% the Update of hyperparameter and covariance of the GPs is carried out inside the initialization of the reducedGP
        BO.ZoomingIn(mu,tub);
    else
        %% Update hyperparameter and covariance of the GPs
        BO.TrainGPs();
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
    %% i use this function to set the parameter of the disitribution that i use 
    %% to sample the starting point for the optimization in the BO.AcqMax function
    BO.SetSampleDistributionParam(diag(tub),[],sigma,sigma_mult,tlb,tub);
    [x_res,y_res] = BO.AcqMax(tlb,tub,transf_for_AcqMax);
    x_candidate = transf(x_res);
    %% TODEBUG
    if(visualization_switch)
        PM.Plot([],1,false,true);
    end
end

function [x_candidate]=GPGlobalExploration(BO,EM_flag,zooming_switch)
    %% remove zooming
    if(zooming_switch)
        BO.ZoomingOut();
    end
    %% Update hyperparameter and covariance of the GPs
    BO.TrainGPs();
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
function [x_candidate,z]=GPLocalExploitation(PM,BO,particle_index,zooming_switch,visualization_switch)
    [mu,V_s,tlb,tub] = PM.GetParticle(particle_index).GetRotTraslBound();
    A  = PM.GetParticle(particle_index).GetCholCov();
    sigma = PM.GetParticle(particle_index).GetSigma();
    sigma_mult = PM.GetParticle(particle_index).GetSigmaMultiplier();
    transf = @(x_)PM.GetParticle(particle_index).RotoTrasl(x_,mu,V_s);
    transf_for_AcqMax = @(x_)PM.GetParticle(particle_index).RotoTraslWithoutSaturation(x_,mu,V_s);
    % insert zooming
    if(zooming_switch)
        %% the Update of hyperparameter and covariance of the GPs is carried out inside the initialization of the reducedGP
        BO.ZoomingIn(mu,tub);
    else
        %% Update hyperparameter and covariance of the GPs
        BO.TrainGPs();
    end
    % here i use eci with the current best perfomance for the particle (only once in a while i use the worst perfomance to reboot eci.it seems to help)
    current_max = PM.GetCurrentLocalMax(particle_index);
    custom_function = @(x_)BO.eci(transf(x_),current_max);
    BO.SetSurrogate('custom',custom_function);
    %% i use this function to set the parameters of the disitribution that i use 
    %% to sample the starting point for the optimization in the BO.AcqMax function
    BO.SetSampleDistributionParam(diag(tub),[],sigma,sigma_mult,tlb,tub);
    [x_res,y_res] = BO.AcqMax(tlb,tub,transf_for_AcqMax);
    x_candidate = transf(x_res);
    % i have removed sigma from the equation to avoid explotion in the
    % covariance of the particle after the boost move
    %z=( A\(x_candidate - mu')' )/sigma;
    z =( A\(x_candidate - mu')');
    z = z';
    %% TODEBUG i removed it because is better to actively zoom in and out when i need (lazy zooming)
    %% TODEBUG
    if(visualization_switch)
        PM.Plot([],particle_index,false,true);
    end
end


function [x_candidate]=GPGlobalExploitation(PM,BO,zooming_switch)
    %% remove zooming
    if(zooming_switch)
        BO.ZoomingOut();
    end
    %% Update hyperparameter and covariance of the GPs
    BO.TrainGPs();
    % with this surrogate i use the current constrained cur max to assure
    % that not cover the real constrained global optimum 
    BO.SetSurrogate('eci',PM.global_maximum_among_particles.cur_max);
    x_candidate = BO.AcqMax();
end
% this is a move that i perform only if i have only one active particle and
% the inaction plus the volume of the gaussian is getting to small to make
% it move. so i basically reinflate the gaussian in the hope that if there
% is something better around the particle im will find it
%% TODO change this function in order to keep the direction of the principle axis of the gaussian 
function str = ReviveParticle(PM)
    str = [];
    cur_particle = PM.GetParticle(1);
    L = cur_particle.GetPrincipalAxes();
    if(max(L)<PM.epsilon)
        str = 'particle reinflated';
        PM.AddParticleInPosition(cur_particle.GetMean(),cur_particle.GetBestPerfomance(),PM.index_map(1),false)
    end
end


%% AD-HOC visualization functions to realize images for the paper


% this function establish in which state currently the system is in
function state = StateMachine(deploy,emergency_swtich,emergency_counter,emergency_event_trigger)
    if(deploy)
        if(emergency_swtich && emergency_counter>emergency_event_trigger)
            state = 'emergency';
        else
            state = 'deploy';
        end
    else
        state = 'optimization';
    end
end


% this function tell us if the current move was succesfull or not (the x_candidate satisify the constraints its fitness function is better than the current best one)
function success = MoveSuccessCheck(act,state,constraints_violation_cost,performances_new,emergency_particle,cur_particle,PM)
     switch state
         case 'deploy'
             if(constraints_violation_cost<0)
                 pls = strcat(state,' action = ',act,' successful!');
                 disp(pls);
                 success = true;
             else
                 pls = strcat(state,' action = ',act,' NOT successful!');
                 disp(pls);
                 success = false;
             end
         case 'emergency'
             if(isempty(emergency_particle))
                 disp('this is the last generation of deploy the next turn im gonna be in emergency')
                 success = false;
             else    
                 %% TOCHECK
                 if(-constraints_violation_cost>emergency_particle.GetBestPerfomance())
                     pls = strcat(state,' action = ',act,' successful!');
                     disp(pls);
                     success = true;
                 else
                     pls = strcat(state,' action = ',act,' NOT successful!');
                     disp(pls);
                     success = false;
                 end
             end
         case 'optimization'
             if(strcmp(act,'local') || strcmp(act,'sample'))
                 if(isempty(PM.GetParticle(cur_particle)))
                     disp('this is the last generation of deploy the next turn im gonna be in optimization')
                     success = false;
                 else    
                     if(constraints_violation_cost<0 && performances_new > PM.GetParticle(cur_particle).GetBestPerfomance())
                         pls = strcat(state,' action = ',act,' successful!');
                         disp(pls);
                         success = true;
                     else
                         pls = strcat(state,' action = ',act,' NOT successful!');
                         disp(pls);
                         success = false;
                     end
                 end
             elseif(strcmp(act,'global'))
                 if(constraints_violation_cost<0 && performances_new > PM.global_maximum_among_particles.cur_max)
                     pls = strcat(state,' action = ',act,' successful!');
                     disp(pls);
                     success = true;
                 else
                     pls = strcat(state,' action = ',act,' NOT successful!');
                     disp(pls);
                     success = false;
                 end
             end
                 
     end
end


function PaperPlot(state,action,emergency_particle,emergency_particle_old,PM,BO,x_candidate,particle_iterator)
    p = 0.95;
    benchmark_x = [14.095000000000038,0.842960789162843];
    % artificial constraints function
    [ymu_a,ys2_a]=BO.gp_s{end-1}.Predict(BO.xl_vis);
    % fitness function 
    [ymu_f,ys2_f]=BO.gp_s{end}.Predict(BO.xl_vis);
    % compute the value for the surrogate function;
    fun = @(x_)BO.surrogate(BO,x_);
    if(strcmp(BO.kind,'custom'))
       not_obsolete_surrogate = true; 
       [X_trasl,Y_trasl] = meshgrid(linspace(BO.temporary_lb(1),BO.temporary_ub(1),BO.res_vis),linspace(BO.temporary_lb(2),BO.temporary_ub(2),BO.res_vis)); 
       xl_trasl = [X_trasl(:) Y_trasl(:)];
       try
           [sur, x_transf] = fun(xl_trasl);
       catch
           disp('obsolete surrogate function due to cancellation of particles, i cannot print this function')
           not_obsolete_surrogate = false;
       end
    else    
       [sur, x_transf] = fun(BO.xl_vis);
    end
    switch state
        %% DEPLOY
        case 'deploy'
            switch action
                % emergency particle last and rectangle and local gp (subplot) 
                case 'local'
                    figure
                    % Plot the objective function
                    subplot(1,2,1),hold on, title('current particle')
                    box on
                    pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
                    plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
                    if(emergency_particle.current_index == 1)
                        emergency_particle_old.PlotBox();
                    else
                        emergency_particle.PlotBox();
                    end
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    subplot(1,2,2),hold on, title('local surrogate')
                    box on
                    pcolor(reshape(x_transf(:,1),BO.res_vis,BO.res_vis),reshape(x_transf(:,2),BO.res_vis,BO.res_vis),reshape(sur,BO.res_vis,BO.res_vis)),shading flat
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                % global GP     
                case 'global'
                    figure
                    subplot(1,3,1),hold on, title('fitness ground truth')
                    pcolor(BO.X_vis,BO.Y_vis,BO.Z_vis),shading flat
                    plot(BO.gp_s{end}.X(1:end,1),BO.gp_s{end}.X(1:end,2), 'kx', 'MarkerSize', 10);
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                    subplot(1,3,2),hold on, title('feasible region ground truth')
                    pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
                    plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                    subplot(1,3,3),hold on, title('global surrogate')
                    pcolor(BO.X_vis,BO.Y_vis,reshape(sur,BO.res_vis,BO.res_vis)),shading flat 
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                % empty         
                case 'sample'
                    disp('none')
            end
            %% full window    
            figure
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            PM.PlotSameWindow(p,'false')
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])    
            %% general plot with all the particle deployed in the same figure
            %% split window
            figure
            subplot(1,2,1),hold on, title('deployed particle')
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
            plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
            PM.PlotSameWindow(p,'false')
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
            subplot(1,2,2),hold on, title('zooming')
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            PM.PlotSameWindow(0.01,'false')
            plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
            plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
        %% EMERGENCY   
        case 'emergency'
            switch action
                % emergency particle last + rectangle and local gp (subplot)
                %% TOCHECK in this case i need to use the older version of the emergency particle before i replace it witih the new one
                case 'local'
                     figure
                     % Plot the objective function
                     subplot(1,2,1),hold on, title('Emergency Particle Search Box')
                     box on
                     pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
                     plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
                     if(emergency_particle.current_index == 1)
                        emergency_particle_old.PlotBox();
                     else
                        emergency_particle.PlotBox(); 
                     end
                     plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                     axis normal;
                     subplot(1,2,2),hold on, title('local surrogate')
                     box on
                     pcolor(reshape(x_transf(:,1),BO.res_vis,BO.res_vis),reshape(x_transf(:,2),BO.res_vis,BO.res_vis),reshape(sur,BO.res_vis,BO.res_vis)),shading flat
                     plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                     axis normal;
                     axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                %new emergency particle strobo and global gp (subplot)     
                case 'global'
                    figure
                    subplot(1,2,1),hold on, title('emergency particle')
                    pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
                    plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
                    if(~isempty(emergency_particle))
                        emergency_particle.Plot();
                    end
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                    subplot(1,2,2),hold on, title('global surrogate')
                    pcolor(BO.X_vis,BO.Y_vis,reshape(sur,BO.res_vis,BO.res_vis)),shading flat
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                % none   
                case 'sample'
                    disp('none')
            end
            %% full window with emergency particle        
            figure
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            if(~isempty(emergency_particle))
                emergency_particle.PlotStrobo(p);
            end
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])        
            
            %% split window with emergency particle and zoom
            figure
            subplot(1,2,1),hold on, title('emergency particle')
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
            plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
            if(~isempty(emergency_particle))
                emergency_particle.PlotStrobo(p);
            end
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
            subplot(1,2,2),hold on, title('zommed')
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            if(~isempty(emergency_particle))
                emergency_particle.PlotStrobo(0.01);
            end
            plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
            plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
        % emergency particle strobo (single plot)    
           
        %% optimization    
        case 'optimization'
            switch action
                % current particle rectangle and local GP (subplot)
                case 'local'
                    % Plot the objective function
                    subplot(1,2,1),hold on, title('Particle Path')
                    box on
                    pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
                    PM.GetParticle(particle_iterator).PlotBox();
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    subplot(1,2,2),hold on, title('Particle Path')
                    box on
                    if(not_obsolete_surrogate)
                        pcolor(reshape(x_transf(:,1),BO.res_vis,BO.res_vis),reshape(x_transf(:,2),BO.res_vis,BO.res_vis),reshape(sur,BO.res_vis,BO.res_vis)),shading flat
                    end
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    axis normal;
                    axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
                % global GP    
                case 'global'
                    figure
                    subplot(1,2,1),hold on, title('emergency particle')
                    pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
                    PM.PlotSameWindow(p,true)
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                    subplot(1,2,2),hold on, title('emergency particle')
                    pcolor(BO.X_vis,BO.Y_vis,reshape(sur,BO.res_vis,BO.res_vis)),shading flat   
                    plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                % none    
                case 'sample'
                    disp('none')
            end   
            % strobo active particles
            figure
            PM.PlotSameWindow(p,true)
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
            % only last position all particles
            figure
            PM.PlotSameWindow(p,false)
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
            %% general plot with all the particle deployed in the same figure
            %% split window
            figure
            subplot(1,2,1),hold on, title('deployed particle')
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
            plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
            PM.PlotSameWindow(p,'false')
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
            subplot(1,2,2),hold on, title('zooming')
            pcolor(BO.X_vis,BO.Y_vis,BO.Z_constr_combined),shading flat
            %% add position of optimal solution
            PM.PlotSameWindow(0.01,'false')
            plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
            plot(benchmark_x(1,1),benchmark_x(1,2), '*k', 'MarkerSize', 10, 'linewidth', 3);
            axis normal;
            axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
%          subplot(1,3,3),hold on, title('real function')
%          pcolor(BO.X_vis,BO.Y_vis,BO.Z_vis),shading flat
%          plot(BO.gp_s{end}.X(1:end,1),BO.gp_s{end}.X(1:end,2), 'kx', 'MarkerSize', 10);
%          axis normal;
%          axis([BO.bounds(1,1),BO.bounds(2,1),BO.bounds(1,2),BO.bounds(2,2)])
         
        otherwise
            warning('something is wrong with the expected state')
    end
end


%% TODO

