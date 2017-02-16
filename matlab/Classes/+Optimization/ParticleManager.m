%% TOADD current global maximum among the particle to see if it makes sense to add a new particle with the global exploitation move

classdef ParticleManager < handle
      
    properties
      particles;  
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
      epsilon            % this number identify which is the codition for which the particle are considerer too close
      inaction_limit     % it defines the longest tolerable series of inaction for a particle
      global_maximum_among_particles % this field is a structure with the current maximum perfomances among all the particles and the index of the particle that holds the best perfomances
    end
    
    methods
        
        function obj = ParticleManager(lambda,size_action,n_constraints,maxAction,minAction,nIterations,explorationRate)
           obj.particles = cell(lambda,1);
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
        end
        
        function [candidate,z] = Sample(obj,particle_index)
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
        
        % after dampling we need to compute the new fitness and the
        % constraints violations
        function UpdateParticle(obj,particle_index,violated_constrained,z,candidate,performances_new)
            obj.particles{particle_index}.Evolve(violated_constrained,z,candidate,performances_new)
        end
        
        function AddParticle(obj,candidate,performance)
            % starting from the beggining of the particles vec fill the
            % first empty space available
            for i=1:length(obj.particles)
                if(isempty(obj.particles{i})) 
                    
                    % selec color 
                    obj.particle_counter = obj.particle_counter + 10;
                    if(obj.particle_counter > length(obj.cmap))
                        obj.particle_counter = 1;
                    end
                    obj.particles{i} = Optimization.Particle(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraints,obj.nIterations,...
                                                             obj.explorationRate,candidate,performance,obj.cmap(obj.particle_counter,:));
                    obj.active_particles = obj.active_particles + 1;
                    break;
                end
            end
        end
        
        function AddParticleInPosition(obj,candidate,performance,index)
            obj.particles{index} = Optimization.Particle(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraints,obj.nIterations,...
                                                     obj.explorationRate,candidate,performance,obj.cmap(obj.particle_counter,:));
            occupied_position = 0;                                     
            for i=1:length(obj.particles);                                     
                if(~isempty(obj.particles{i}))
                    occupied_position = occupied_position + 1;
                end
            end
            obj.active_particles = occupied_position;
        end
        
        function DeleteParticle(obj,index,cause)
            % change the status of particle
            obj.particles{index}.status = cause;
            % store the particle 
            obj.old_particles{end + 1} = obj.particles{index};
            obj.particles{index} = [];
            obj.active_particles = obj.active_particles - 1;
        end
        
        % remove redudant particles that are heading to the same local maxima / minima 
        % remove particle that got stuck in some minima / maxima
        %% TOADD in this move i have to add a rule that not kill particle for inaction when they are the best 
        %% this avoid killing particle that show to be the best of the batch (there is no point to remain wiht one particle that is around some local optima)
        %% in this way in the end i will be with one particles that is the absoulte best
        function PruneParticles(obj)
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
                                    disp('delete because is too close!')
                                    % im cancelling the current particle that im comparing with the other so i need to stop the inner for
                                    obj.DeleteParticle(i,'closeness');
                                    break;
                                else
                                    disp('delete because is too close!')
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
                        disp('delete because is inactive for too long!')
                        obj.DeleteParticle(ii,'inaction');
                    end
                end
            end
            %obj.DeleteParticle(to_delete);
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