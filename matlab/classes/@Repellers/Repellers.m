classdef  Repellers < handle
    
   properties
      target_link;     % vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
      type;            % cartesian_x,cartesian_rpy, joint vector  
      mask;            % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      J_rep_func       % this specify wich kind of function i want to
      obstacle_ref     % vector of index that specify for each repellers which is the obstacle  G_OB (global vector of Obstacle) associated to the current repeller  
      task_dimension   % vector that define the dimension of each repulsive task
      repellers_fun;   % cell array of func of repellers of type (i,j) where j is the task of the i-th kinematic chain
      Jac_rep          % cell array of the final jacobian is the final jacobian of the repellers
                    
   end
       
    
   methods
      %#TODO add control of the input
      function obj = Repellers(target_link,type,mask,type_of_J_rep,obstacle_ref,chain_dof) 
          
         obj.target_link = target_link;
       
         if(getnameidx({'joint' 'cartesian_x' 'cartesian_rpy'} , type) ~= 0 )
            obj.type = type;
         end
         obj.mask = mask;
         obj.obstacle_ref = obstacle_ref;
         %Jac_rep = zeros()
      end       
         
      function GetJacob(obj,cp,J,chain,task)
           %Jac_rep{chain}(,:) = 
      end
      
      function J_rep = DirectionCartesian(obj,cp,J,chain,task)
      end
   end
    
end