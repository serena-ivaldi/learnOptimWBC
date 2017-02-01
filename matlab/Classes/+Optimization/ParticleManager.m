classdef ParticleManager < handle
      
    properties
      particles;  
      old_particles;     % store particles that are dead for some reason 
      lambda;            % maximum number of particles allowed 
      active_particles;
      size_action;  
      n_constraints; 
      maxAction;
      minAction;
      nIterations;
      explorationRate;
      cmap;              % set of color to give to the particle
      particle_counter   % counter that is used to assign color
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
           obj.particle_counter = 0;
        end
        
        function [candidate,z] = Sample(obj,particle_index)
           z =  mvnrnd(zeros(1, obj.size_action), eye(obj.size_action));
           candidate = obj.particles{particle_index}.GetMean() + obj.particles{particle_index}.GetSigma() *(obj.particles{particle_index}.GetCholCov() * z')'; 
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
        
        function DeleteParticle(obj,index)
            % store the particle 
            obj.old_particles{end + 1} = obj.particles{index};
            obj.particles{index} = [];
            obj.active_particles = obj.active_particles - 1;
        end
        
        % remove redudant particles that are heading to the same local maxima / minima 
        % remove particle that got stuck in some minima / maxima
        function PruneParticles(obj)
            %% do some checking on the trajectory of the particle (or the current mean and covariance)
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
        
        
        
        function ret = RotoTrasl(obj,x,mu,V_s)
           % to allow this function to work with multiple input i had to
           % rearrange a little bit the trasfromation 
           % basically im doing  ----> ret = mu + R*x   with  R = V_s
           mu = repmat(mu',size(x,1),1);
           ret = mu + x*V_s';
           % saturation 
           minaction = repmat(obj.minAction,size(ret,1),1);
           maxaction = repmat(obj.maxAction,size(ret,1),1);
           %ret(1, ret(1,:) > obj.maxAction) = obj.maxAction(ret(1,:) > obj.maxAction);
           %ret(1, ret(1,:) < obj.minAction) = obj.minAction(ret(1,:) < obj.minAction);
           ret(ret < minaction) = minaction(ret < minaction);
           ret(ret > maxaction) = maxaction(ret > maxaction);
            
        end
        
        % with this function i plot the current particle with their own
        % covariance
        function Plot(obj)
            
            % compute how many subploat
            img_col  = 4; % i set 4 col to have enough space for the fitness function visualization 
            img_rows = ceil((obj.lambda)/img_col);  % i have 6 column i have to add rows depending on the number of total particle
            subplot_position = 1;
            figure
            for counter = 1:obj.lambda
                if(~isempty(obj.particles{counter}))     
                    subplot(img_rows,img_col,subplot_position), hold on, title(sprintf('particle position %.2e', counter))
                    box on
                    obj.particles{counter}.PlotTrace();
                    obj.particles{counter}.Plot();
                    subplot_position = subplot_position + 1;
                    
                    axis normal ;
                    axis([obj.minAction(1,1),obj.maxAction(1,1),obj.minAction(1,2),obj.maxAction(1,2)])
                end
            end
             
        end
        
        
    end
    
    
    
end