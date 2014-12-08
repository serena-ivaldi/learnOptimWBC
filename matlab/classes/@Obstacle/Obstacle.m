% this class can manage wall and repulsive point

classdef Obstacle < handle
    
    properties
      description % one point for repeller matrix for wall 
      type        % 'wall'  with collision detection or 'repulsor' 
      tol         %  threshold for collision detection 
    end
       
    
   methods
      %#TODO add control of the input
      function obj = Obstacle(description,type,tol) % i can specify through varargin the time duration of the sampled trajectories 
          
         obj.description = description;
       
         if(getnameidx({'wall' 'repulsor'} , type) ~= 0 )
            obj.type = type;
         end
         
         obj.tol = tol;        
      end 
      
      
      
      function dist = Dist(obj,cp,L)
      
          if(strcmp(obj.type,'repellers'))
             dist = norm((cp - obj.description),L);
          elseif(strcmp(obj.type,'wall'))
             dist = obj.MinDist(cp,L);
          end
      
      end
      
        function dist = MinDist(obj,cp,L)
      
          if(L==1)
              dist_matrix = abs((cp(1,1) - obj.description.X)) + abs((cp(1,2) - obj.description.Y)) + abs((cp(1,3) - obj.description.Z));
              dist = min(dist_matrix(:)); 
          elseif(L==2)
              dist_matrix = (cp(1,1) - obj.description.X).^2 + (cp(1,2) - obj.description.Y).^2 + (cp(1,3) - obj.description.Z).^2;
              dist = min(dist_matrix(:)); 
          end

        end
      
      
      
   end   
    
    
   
    
    
    
end