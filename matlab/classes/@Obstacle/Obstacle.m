% this class can manage wall and repulsive point

classdef Obstacle < handle
    
    properties
      description % one point for repeller matrix for wall 
      type        % 'wall'  with collision detection or 'repeller' 
      tol         %  threshold for collision detection 
    end
       
    
   methods
   %#TODO add control of the input
            function obj = Obstacle(description,type,tol) % i can specify through varargin the time duration of the sampled trajectories 

               obj.description = description;
               if(getnameidx({'wall' 'repeller'} , type) ~= 0 )
                  obj.type = type;
               end

               obj.tol = tol;        
            end 
      
            function point=GetDescription(obj)
                if(strcmp(obj.type,'repeller'))
                   point = obj.description';
                  % add wall case with norm
      %           elseif(strcmp(obj.type,'wall'))
      %              point = obj.MinDist(cp,L);
                end
            end    

            function dist = Dist(obj,cp,L)

                if(strcmp(obj.type,'repeller'))
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
                  dist = sqrt(dist);
              end

            end
      
      
            function n = Normal(obj,cp) 
              if(strcmp(obj.type,'repeller'))
                 n = (cp - obj.description)/norm((cp - obj.description));
              elseif(strcmp(obj.type,'wall'))
                 n = obj.MinDistNorm(cp);
              end
            end
        
            function n = MinDistNorm(obj,cp)
                  dist_matrix = (cp(1,1) - obj.description.X).^2 + (cp(1,2) - obj.description.Y).^2 + (cp(1,3) - obj.description.Z).^2;
                  [dist i] = min(dist_matrix(:)); 
                  dist = sqrt(dist);
                  point = [obj.description.X(i); obj.description.Y(i); obj.description.Z(i)];
                  n = (cp' - point)/dist;   
            end         
           
            
            
                   
   end
        
      
end   
    
    
   
    
    
    
