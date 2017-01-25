classdef ParticleManager < handle
      
    properties
      particles;  
      lambda; % maximum number of particles allowed 
      active_particles;
      size_action;  
      n_constraints  
      maxAction;
      minAction;
      nIterations;
      explorationRate;
      steptogo;
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
        
        
        
        function AddParticle(obj)
            % starting from the beggining of the particles vec fill the
            % first empty space available
            for i=1:length(obj.particles)
                if(~isempty(obj.particles{i}))
                    obj.particles{i} = Optimization.ParticleExperimental(obj.size_action,obj.maxAction,obj.minAction,obj.n_constraint,obj.steptogo,obj.explorationRate);
                    obj.active_particles = obj.active_particles + 1;
                    break;
                end
            end
        end
        
        function DeleteParticle(obj,index)
            obj.particles{index} = [];
            obj.active_particles = obj.active_particles - 1;
        end
        
        % remove redudant particles that are heading to the same local maxima / minima 
        function PruneParticles(obj)
        end
        
        function UpdateParticle(obj,index)
        end
        
        % return index of active particles and information about the status
        % of each of them (ex: current max)
        function GetParticleInfo()
        end
        
        
    end
    
    
    
end