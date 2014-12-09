
classdef  Repellers < handle
    
   properties
      target_link;     % vector that define wich kind of link  one for every kinematic chain
      type;            % cartesian_x,cartesian_rpy, joint vector  
      mask;            % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      J_rep_func       % this specify wich kind of function i want to use to compute the repellers jacobian
      obstacle_ref     % vector of index that specify for each repellers which is the obstacle  G_OB (global vector of Obstacle) associated to the current repeller  
      chain_dof        % vector of total dimension for each kinematic chain
      task_dimension   % vector that define the cumulative dimension for each repulsive task. i use this as a vector of pointer to build the jacobian
      repellers_fun;   % cell array of func of repellers of type (i,j) where j is the task of the i-th kinematic chain
      Jac_rep          % cell array of the final jacobian is the final jacobian of the repellers
                    
   end
       
    
   methods
      %type_of_J_rep specify the kind of repellers that i want to use
      %chain_dof is a vector with the dimension 
      function obj = Repellers(target_link,type,mask,type_of_J_rep,obstacle_ref,chain_dof) 
          
         obj.target_link = target_link;
       
         if(getnameidx({'joint' 'cartesian_x' 'cartesian_rpy'} , type) ~= 0 )
            obj.type = type;
         end
         obj.mask = mask;
         obj.obstacle_ref = obstacle_ref;
         obj.chain_dof = chain_dof;
         % in this cycle i pre allocate the jacobian for each chain and 
         % i compute from mask the dimension of each task
         dim = 0;
         for i=1:size(mask,1)
             for j=1:size(mask,2)   
                 dim = dim + nnz(mask{i,j});   
                 obj.task_dimension(i,j) = nnz(mask{i,j});
             end
             obj.Jac_rep{i} = zeros(dim,chain_dof(i));
             dim = 0;
         end
         % in this cycle i build the vector of function handle to compute
         %  repellers jacobian
         for i=1:size(mask,1)
             for j=1:size(mask,2)
                obj.repellers_fun(i,j) = eval(strcat('@',type_of_J_rep));
             end
             
         end
         
         
      end       
         
      function SetJacob(obj,direct_kin,J,chain,task)
           if(task==1)
               obj.Jac_rep{chain}(1:obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun(chain,task),obj,direct_kin,J,chain,task);
           else
               obj.Jac_rep{chain}(obj.task_dimension(chain,task - 1) + 1:obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun(chain,task),obj,direct_kin,J,chain,task); 
           end
      end
      
      function J_rep = DirectionCartesian(obj,direct_kin,J_old,chain,task)
      global G_OB;    
          if(strcmp(type{chain,task},'cartesian_x'))
            J = ReshapeJacobian(J_old,[],obj.chain_dof(chain),obj.target_link(chain,task),obj.mask{chain,task},'trans');
          elseif(strcmp(type{chain,task},'cartesian_rpy'))
            J = ReshapeJacobian(J_old,[],obj.chain_dof(chain),obj.target_link(chain,task),obj.mask{chain,task},'rot');    
          end
          v = 2*(direct_kin - G_OB{obj.obstacle_ref(chain,task)}).^(2); 
          J_rep = diag(v)*J;
          
      end
   end
    
end