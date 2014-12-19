
classdef  Repellers < handle
    
   properties
      chain_dof        % number of degrees of freeedom for each kinematic chain 
      target_link;     % vector that define wich kind of link interact with the repellers,one for every kinematic chain
      type;            % cartesian_x,cartesian_rpy, joint vector  
      mask;            % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      J_rep_func       % this specify wich kind of function i want to use to compute the repellers jacobian
      obstacle_ref     % vector of index that specify for each repellers which is the obstacle  G_OB (global vector of Obstacle) associated to the current repeller  
      task_dimension   % vector that define the cumulative dimension for each repulsive task. i use this as a vector of pointer to build the jacobian
      repellers_fun;   % cell array of function of repellers of type (i,j) where j is the task of the i-th kinematic chain
      Jac_rep          % cell array of the final jacobian is the final jacobian of the repellers
                    
   end
       
    
   methods
      %type_of_J_rep specify the kind of repellers that i want to use
      %chain_dof is a vector with the dimension 
      function obj = Repellers(chain_dof,target_link,type,mask,type_of_J_rep,obstacle_ref) 
          
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
                 obj.task_dimension(i,j) = dim;
             end
             obj.Jac_rep{i} = zeros(dim,chain_dof(i));
             dim = 0;
         end
         % in this cycle i build the vector of function handles to compute
         %  repellers jacobian
         for i=1:size(mask,1)
             for j=1:size(mask,2)
                obj.repellers_fun{i,j} = eval(strcat('@',type_of_J_rep{i,j}));
             end
             
         end
         
         
      end    
      
      
      
      function n=GetTotalDimRep(obj,cur_chain)
          n=obj.task_dimension(cur_chain,end);
      end
      
      
      
      function n=GetNumTasks(obj,cur_chain)
          n=size(obj.target_link{cur_chain},2); 
      end
      
      
      function N=ComputeProjector(obj,cur_chain,cur_chain_dim,number_of_tasks_for_cur_chain,alpha,t) 
          
          n_of_alpha_repellers=size(alpha(cur_chain,:),2);
          alpha_vec=zeros( n_of_alpha_repellers - number_of_tasks_for_cur_chain,1);
          index = 1;
          for i = number_of_tasks_for_cur_chain+1:n_of_alpha_repellers
 
            alpha_vec(index) = alpha{cur_chain,i}.GetValue(t);
            index = index + 1;
          end
          alpha_diag = diag(alpha_vec);
          I = eye(cur_chain_dim);
          [~,~,V] = svd(obj.Jac_rep{cur_chain},'econ');
          N = (I-V*alpha_diag*V');
            
      end
      
      
      % ALL THE FUNCTION BELOW WORK ONLY WITH cartesian_x REPELLERS
      function SetJacob(obj,cur_rob,q,qd,chain,task)
           
          [J,~,x]=obj.DirKin(cur_rob,q,qd,chain,task);

          if(task==1)
               obj.Jac_rep{chain}(1:obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task);
           else
               obj.Jac_rep{chain}(obj.task_dimension(chain,task - 1) + 1:obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task); 
           end
      end
      
      function J_rep = DirectionCartesian(obj,direct_kin,J_old,chain,task)
        global G_OB;    
          if(strcmp(obj.type{chain,task},'cartesian_x')) 
            J = ReshapeJacobian(J_old,[],obj.chain_dof(chain),obj.target_link{chain,task},obj.mask{chain,task},'trans');
          elseif(strcmp(obj.type{chain,task},'cartesian_rpy'))
            %J = ReshapeJacobian(J_old,[],obj.chain_dof(chain),obj.target_link(chain,task),obj.mask{chain,task},'rot');    
          end
          v = 2*(direct_kin - G_OB(obj.obstacle_ref(chain,task)).GetDescription()).^(2);
          J_rep = diag(v)*J;
          
      end
   end
   
   
   
    
end