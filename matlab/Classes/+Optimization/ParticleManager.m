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
                    obj.particles{i} = Optimization.Particle(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraints,obj.nIterations,obj.explorationRate,candidate,performance);
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
        
        % with this function i plot the current particle with their own
        % covariance
        function Plot(obj)
            figure
            for i=1:length(obj.particles)
                if(~isempty(obj.particles{i}))               
                    obj.particles{i}.Plot();
                end
            end  
        end
    end
    
    
    
end