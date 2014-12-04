% vanilla version
classdef  Repellers < handle
    
   properties
      target_link      %  vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
      type;            % cartesian_x,cartesian_rpy, joint vector  
      mask;            % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
     
                    
   end
       
    
   methods
      %#TODO add control of the input
      function obj = Repellers(target_link,type,mask,varargin) % i can specify through varargin the time duration of the sampled trajectories 
          
         obj.target_link = target_link;
       
         if(getnameidx({'joint' 'cartesian_x' 'cartesian_rpy'} , type) ~= 0 )
            obj.type = type;
         end
         obj.mask = mask;        
      end       
      
   end
    
end