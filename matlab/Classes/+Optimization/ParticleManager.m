%% TOADD current global maximum among the particle to see if it makes sense to add a new particle with the global exploitation move

classdef ParticleManager < handle
      
    properties
      particles;  
      index_map          % necessary field to map from a progressive iterator (1,2,3...) to the acutal position of the particle in the particles vector (it is necessary for sample and update particle that take the index from outside)
      old_particles;     % store particles that are dead for some reason 
      lambda;            % maximum number of particles allowed 
      active_particles;  % number of active particles (a number between zero and lambda)
      size_action;  
      n_constraints; 
      maxAction;
      minAction;
      nIterations;
      explorationRate;
      cmap;              % set of color to give to the particle
      particle_counter   % counter that is used to assign color
      epsilon            % this number identify which is the condition in which the particle are considerer too close
      inaction_limit     % it defines the longest tolerable series of inaction for a particle
      global_maximum_among_particles % this field is a structure with the current maximum perfomances among all the particles and the index of the particle that holds the best perfomances
    end
    
    methods
        
        function obj = ParticleManager(lambda,size_action,n_constraints,maxAction,minAction,nIterations,explorationRate)
           obj.particles = cell(lambda,1);
           obj.index_map = [];
           obj.active_particles = 0;
           obj.lambda = lambda;
           obj.size_action = size_action;
           obj.n_constraints = n_constraints;
           obj.maxAction = maxAction;
           obj.minAction = minAction;
           obj.nIterations = nIterations;
           obj.explorationRate = explorationRate;      
           obj.cmap = hsv(100);
           obj.particle_counter = 1;
           c = 0.1;
           obj.epsilon = obj.conf2mahal(c, size_action);
           % number of turn that the particle is allowed to not improve
           % before deletion 
           %% TODO define it as a parameter of the method
           obj.inaction_limit = 10;
           obj.global_maximum_among_particles.cur_max = -inf;
           obj.global_maximum_among_particles.cur_best_particle = 0;
           obj.global_maximum_among_particles.cur_best_action = zeros(1,size_action);
        end
        
        %% ( i need a conversion from particle 1 to the actual position of the first particle in the particle array)
        function [candidate,z] = Sample(obj,cur_index)
           %% index conversion
           particle_index = obj.index_map(cur_index); 
           z =  mvnrnd(zeros(1, obj.size_action), eye(obj.size_action));
           candidate = obj.particles{particle_index}.GetMean() + obj.particles{particle_index}.GetSigma() *(obj.particles{particle_index}.GetCholCov() * z')';
           %% DEBUG
           if(~isreal(candidate))
               disp('error x not real');
           end
           % saturation
           candidate(1, candidate(1,:) > obj.maxAction) = obj.maxAction(candidate(1,:) > obj.maxAction);
           candidate(1, candidate(1,:) < obj.minAction) = obj.minAction(candidate(1,:) < obj.minAction);
        end
        
        function UpdateParticle(obj,cur_index,violated_constrained,z,candidate,performances_new,update_max)
            %% index conversion
            particle_index = obj.index_map(cur_index);
            obj.particles{particle_index}.Evolve(violated_constrained,z,candidate,performances_new);
            %% update best solution (only if is the right fitness function)
            if(update_max)
                obj.UpdateCurrentMax(particle_index);    
            end
        end
        
        function particle = GetParticle(obj,cur_index)
            particle_index = obj.index_map(cur_index);
            particle = obj.particles{particle_index};
        end
        
        function UpdateCurrentMax(obj,index)
            if(obj.particles{index}.GetBestPerfomance()>obj.global_maximum_among_particles.cur_max)
                obj.global_maximum_among_particles.cur_max = obj.particles{index}.GetBestPerfomance();
                obj.global_maximum_among_particles.cur_best_particle = index;
                obj.global_maximum_among_particles.cur_best_action = obj.particles{index}.GetMean();
            end
        end
        %% in this function i made the assumption that index_map is in "ascendind order"
        function UpdateIndMap(obj,index,add_or_delete)
            if(strcmp(add_or_delete,'add'))
                updated = false;
                % initialization of the vector 
                if(isempty(obj.index_map))
                    obj.index_map(1) = 1;
                    updated = true;
                % at the beginngin we check if the index is lower than the
                % first element or higher than the last
                elseif(index < obj.index_map(1))
                    obj.index_map = [index;obj.index_map];
                    updated = true;
                elseif(index > obj.index_map(end))
                    obj.index_map = [obj.index_map;index]; 
                    updated = true;
                elseif( ~(prod(index ~= obj.index_map)) ) % this line is a tricky way to verify if the index is already in the index_map if it is true i have to do nothing
                    updated = true;
                end
                if(~updated)
                    for ii = 1:length(obj.index_map)
                        if(index < obj.index_map(ii))
                            obj.index_map = [obj.index_map(1:ii-1);index;obj.index_map(ii:end)];
                            break;
                        end
                    end
                end
            elseif(strcmp(add_or_delete,'delete'))
                obj.index_map(index==obj.index_map) = [];
            end
        end
        
        function AddParticle(obj,candidate,performance,update_max)
            added_particle = false;
            %% starting from the beggining of the particles vec fill the
            %% first empty space available
            for i=1:obj.lambda
                if(isempty(obj.particles{i}))  
                    % selec color 
                    obj.particle_counter = obj.particle_counter + 10;
                    if(obj.particle_counter > length(obj.cmap))
                        obj.particle_counter = 1;
                    end
                    obj.particles{i} = Optimization.Particle(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraints,obj.nIterations,...
                                                             obj.explorationRate,candidate,performance,obj.cmap(obj.particle_counter,:));
                    obj.active_particles = obj.active_particles + 1;
                    added_particle = true;
                    %% update ind_map
                    obj.UpdateIndMap(i,'add');
                    %% update best solution (only if is the right fitness function)
                    if(update_max)
                        obj.UpdateCurrentMax(i);    
                    end
                    break;
                end
            end
            %% if im adding a new particle but the particles is full i allow
            %% to extend the particle list by one
            if(~added_particle)
                 obj.particles{end + 1} = Optimization.Particle(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraints,obj.nIterations,...
                                                             obj.explorationRate,candidate,performance,obj.cmap(obj.particle_counter,:));
                 obj.active_particles = obj.active_particles + 1;
                 % i have to update lambda on top of 
                 obj.lambda = obj.lambda + 1;
                 %% update ind_map
                 obj.UpdateIndMap(obj.lambda,'add');
                 %% update best solution (only if is the right fitness function)
                 if(update_max)
                    obj.UpdateCurrentMax(obj.lambda);   
                 end
                 
            end
            
            
            
        end
        %% i use this function when i want to control exactly in which slot i will place the new particle 
        %% i dont care if the final position is taken or not after that i just recount the number of occupied slot to update the 
        %% the current active_particles field
        %% this function is used during the emergency mode
        %% with update_max i control if im going to update the current max solution or not
        function AddParticleInPosition(obj,candidate,performance,index,update_max)
            obj.particles{index} = Optimization.Particle(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraints,obj.nIterations,...
                                                     obj.explorationRate,candidate,performance,obj.cmap(obj.particle_counter,:));
            cur_num_of_active_particles= 0;                                     
            for i=1:length(obj.particles);                                     
                if(~isempty(obj.particles{i}))
                    cur_num_of_active_particles = cur_num_of_active_particles + 1;
                end
            end
            obj.active_particles = cur_num_of_active_particles;
            %% update ind_map
            obj.UpdateIndMap(index,'add');
            %% update best solution (only if is the right fitness function)
            if(update_max)
               obj.UpdateCurrentMax(index);    
            end
        end
        
        function DeleteParticle(obj,index,cause)
            %% change the status of particle
            obj.particles{index}.status = cause;
            %% update ind_map
            obj.UpdateIndMap(index,'delete');
            %% store the particle only if is not empty
            if(~isempty(obj.particles{index}));
                obj.old_particles{end + 1} = obj.particles{index};
                obj.active_particles = obj.active_particles - 1;
                obj.particles{index} = [];
            end
            
            
        end
        
        % remove redundant particles that are heading to the same local maxima / minima 
        % remove particle that got stuck in some minima / maxima
        % in this move i added a rule that not kill particle for inaction when they are the best 
        % this avoid killing particle that show to be the best of the batch (there is no point to remain wiht one particle that is around some local optima)
        % in this way in the end i will be with one particles that is the absoulte best
        % there is no need to to manage the empty slot problem here because
        % i iterate through all the slot at the end of a full sweept
        function str = PruneParticles(obj)
            str = [];
            if(obj.active_particles >1)
                %% closeness (check the distance between candidates and remove the close one)
                for i=1:obj.lambda - 1
                    if(~isempty(obj.particles{i}))
                        cur_particle_pos = obj.particles{i}.GetMean();
                        for j = i+1:obj.lambda
                            if(~isempty(obj.particles{j}))
                                compare_particle_pos = obj.particles{j}.GetMean();
                                dist = norm(cur_particle_pos-compare_particle_pos);
                                if(dist<=obj.epsilon)
                                    if(obj.particles{j}.GetBestPerfomance() >= obj.particles{i}.GetBestPerfomance())
                                        str ='delete because is too close!';
                                        % im cancelling the current particle that im comparing with the other so i need to stop the inner for
                                        obj.DeleteParticle(i,'closeness');
                                        break;
                                    else
                                        str='delete because is too close!';
                                        % i delete the other particle so i keep searching using the current_particle (the one specified by i index)
                                        obj.DeleteParticle(j,'closeness');
                                    end
                                end
                            end
                        end
                    end
                end
                %% non-evolution (check if the particle is stagnant for too long)
                for ii= 1:obj.lambda
                    if(~isempty(obj.particles{ii}))
                        if(obj.particles{ii}.turns_of_inaction>obj.inaction_limit)
                            if(ii~=obj.global_maximum_among_particles.cur_best_particle)  
                                str = 'delete because is inactive for too long!';
                                obj.DeleteParticle(ii,'inaction');
                            end
                        end
                    end
                end
            end
        end
        
        % return index of active particles and information about the status
        % of each of them (ex: current max)
        function info = GetParticleInfo(obj)          
            for i=1:length(obj.particles)
                if(~isempty(obj.particles{i}))
                    cur_info.ind = i;
                    cur_info.y_max = obj.particles{i}.GetBestPerfomance();
                    cur_info.mean  = obj.particles{i}.GetMean();
                    cur_info.C     = obj.particles{i}.GetCov();
                    info(end + 1) = cur_info;
                end
            end
            
        end
        
        function [func, tlb, tub] = GetRotTraslFuncAndBound(obj,particle_index)
            [mu,V_s,tlb,tub] = obj.particles{particle_index}.GetRotTraslBound();
            func = @(x_)obj.particles{particle_index}.RotoTrasl(x_,mu,V_s);
        end
        
        
        % with this function i plot the current particle with their own
        % covariance
        function Plot(obj,x_candidate,current_particle,plot_new_candidate,plot_box)
            % compute how many subplot
            img_col  = 4; % i set 4 col to have enough space for the fitness function visualization 
            img_rows = ceil((obj.lambda)/img_col);  % i have 6 column i have to add rows depending on the number of total particle
            subplot_position = 1;
            if(obj.active_particles>0)
                figure
            end
            for counter = 1:obj.lambda
                if(~isempty(obj.particles{counter}))     
                    subplot(img_rows,img_col,subplot_position), hold on, title(sprintf('particle position %.2e', counter))
                    box on
                    if(plot_box)
                        if(~isempty(current_particle))
                            if(counter == current_particle)
                                obj.particles{counter}.PlotBox()
                            end
                        end
                    end
                    obj.particles{counter}.PlotTrace();
                    obj.particles{counter}.Plot();
                    if(plot_new_candidate)
                        if(~isempty(current_particle))
                            if(counter == current_particle)
                                plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                            end
                        end
                    end
                    subplot_position = subplot_position + 1;
                    
                    axis normal ;
                    axis([obj.minAction(1,1),obj.maxAction(1,1),obj.minAction(1,2),obj.maxAction(1,2)])
                end
            end
             
        end
        
        
    end
    
    
    methods(Static)
        % Inputs:
        %
        %   c    - the confidence interval
        %   d    - the number of dimensions of the Gaussian distribution
        %
        % Outputs:
        %
        %   m    - the Mahalanobis radius of the ellipsoid enclosing the
        %          fraction c of the distribution's probability mass
        %
        function m = conf2mahal(c, d)
                %% TODO i have got this computation from a resource online, not fully sure.
                m = sqrt(chi2inv(c, d)); % matlab stats toolbox
                % pr = 0.341*2 ; c = (1 - pr)/2 ; norminv([c 1-c],0,1)

    %             pr = c ; c = (1 - pr)/2 ; 
    %             m = norminv([c 1-c],0,1) ;
    %             m = m(2) ;
        end        
    end
    
end