
classdef  Repellers < handle
    
   properties
      chain_dof            % number of degrees of freeedom for each kinematic chain 
      target_link;         % vector that define wich kind of link interact with the repellers,one for every kinematic chain
      type;                % cartesian_x,cartesian_rpy, joint vector  
      mask;                % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      J_rep_func           % this specify wich kind of function i want to use to compute the repellers jacobian
      obstacle_ref         % vector of index that specify for each repellers which is the obstacle  G_OB (global vector of Obstacle) associated to the current repeller  
      task_dimension       % vector that define the cumulative dimension for each repulsive task. i use this as a vector of pointer to build the jacobian. If single_alpha = 1 use only one alpha for the whole controller
      single_alpha         % is a vector of vector of flag 0 and 1 that define for each repellers if i want one alpha for each dimension of my repellers or only one alpha
      type_of_rep_strct    % with this parameter i can specify wich kind of repellers structure i want to use : 'extended' or 'stacked' (column vector of string)  
      n_alpha_x_chain      % column vector that on the basis of the single_alpha contain the number of "different" alpha for repellers per kinematic chain 
      map_from_alpha_to_rep% this vector has the dimension of the number of alpha but say to me at wich repellors the alpha are reffered so is basically a vector of pointers, one for each chain 
      repellers_fun;       % cell array of function of repellers of type (i,j) where j is the task of the i-th kinematic chain
      Jac_rep              % cell array of the final jacobian is the final jacobian of the repellers
                    
   end
       
    
   methods
      %type_of_J_rep specify the kind of repellers that i want to use
      %chain_dof is a vector with the dimension 
      function obj = Repellers(chain_dof,target_link,type,mask,type_of_J_rep,obstacle_ref,single_alpha,type_of_rep_strct) 
          
         obj.target_link = target_link;
       
         if(getnameidx({'joint' 'cartesian_x' 'cartesian_rpy'} , type) ~= 0 )
            obj.type = type;
         end
         obj.mask = mask;
         obj.obstacle_ref = obstacle_ref;
         obj.chain_dof = chain_dof;
         % in this cycle i copy the value that i have in RuntimeVariable
         % inside the object in such a way that i dont care about the
         % structure of  single_alpha in RuntimeVariable. the only things that
         %  is necesessary is that the single_alpha is sufficiently long.
         for i=1:size(mask,1)
             for j=1:size(mask{i},2) 
                obj.single_alpha{i}(1,j) = single_alpha{i}(1,j);
             end
         end
         
         obj.type_of_rep_strct = type_of_rep_strct;
         % in this cycle i compute the number of alpha for each kinematic
         % chain for the repellers
         for i=1:size(mask,1)
            obj.n_alpha_x_chain(i)=obj.ComputeNumberOfWeightFuncRep(i);    
         end
         for i=1:size(mask,1)
             switch obj.type_of_rep_strct{i}
                 case 'extended' 
                     % in this cycle i pre allocate the jacobian for each chain and 
                     % i compute from mask the dimension of each task
                     dim = 0;
                         for j=1:size(mask{i},2)
                             dim = dim + nnz(mask{i,j});  
                             obj.task_dimension(i,j) = dim;
                         end
                     obj.Jac_rep{i} = zeros(dim,chain_dof(i));
                 case 'stacked'
                     obj.Jac_rep{i} = cell(1,obj.GetNumberOfWeightFuncRep(i));
                 otherwise 
                error('Unexpected structure for repulsor');
             end  
         end
         % in this cycle i build the vector of function handles to compute
         %  repellers jacobian
         for i=1:size(mask,1)
             for j=1:size(mask,2)
                obj.repellers_fun{i,j} = eval(strcat('@',type_of_J_rep{i,j}));
             end
             
         end
         
         
      end    
       
      function n=ComputeNumberOfWeightFuncRep(obj,cur_chain)
          
           n = 0;
           index = 1;
           for j=1:size(obj.mask{cur_chain},2) 
               if (obj.single_alpha{cur_chain}(j)) 
                    n = n + 1;
                    obj.map_from_alpha_to_rep{cur_chain}(index) = j;
                    index = index + 1;
               else
                    n = n + nnz(obj.mask{cur_chain,j});
                    for k = 1 : nnz(obj.mask{cur_chain,j})
                        obj.map_from_alpha_to_rep{cur_chain}(index) = j;
                        index = index + 1;
                    end
               end
           end         
      end
      
      function n=GetNumberOfWeightFuncRep(obj,cur_chain)
          n=obj.n_alpha_x_chain(cur_chain);
      end
      
      
      function n=GetNumTasks(obj,cur_chain)
          n=size(obj.target_link{cur_chain},2); 
      end
      
      
      function N=ComputeProjector(obj,cur_chain,cur_chain_dim,alpha,t)       
        
        n_of_alpha_repellers=obj.GetNumberOfWeightFuncRep(obj,cur_chain);  
          
        switch obj.type_of_rep_strct{cur_chain}

            case 'extended' 
                
                alpha_vec=zeros(n_of_alpha_repellers,1);
                index_alpha_vec = 1;
                for index_map = 1:n_of_alpha_repellers;
                    if (obj.single_alpha{cur_chain}(index_map))
                        % i have to use a different index that maps the
                        % current number of alpha to the original repellers
                        for k=1:nnz(obj.mask{cur_chain,obj.map_from_alpha_to_rep{cur_chain}(index_map)})
                            alpha_vec(index_alpha_vec)=alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t);
                            index_alpha_vec = index_alpha_vec + 1; 
                        end
                    else
                        alpha_vec(index_alpha_vec) = alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t);
                        index_alpha_vec = index_alpha_vec + 1;    
                    end
                end
                alpha_diag = diag(alpha_vec);
                I = eye(cur_chain_dim);
                [~,~,V] = svd(obj.Jac_rep{cur_chain},'econ');
                N = (I-V*alpha_diag*V');
            case 'stacked' 
  
                for index_map = 1:n_of_alpha_repellers
                   alpha_vec = zeros(1,nnz(obj.mask{cur_chain}{index_map},2));    
                   if (obj.single_alpha{cur_chain}(index_map))
                        % i have to use a different index that maps the
                        % current number of alpha to the original repellers
                        for k=1:nnz(obj.mask{cur_chain,obj.map_from_alpha_to_rep{cur_chain}(index_map)})
                            alpha_vec(k)=alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t);
                            
                        end
                   else 
                        for k=1:nnz(obj.mask{cur_chain,obj.map_from_alpha_to_rep{cur_chain}(index_map)})
                            alpha_vec(k) = alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t);   
                        end
                   end
                   alpha_diag = diag(alpha_vec);
                   I = eye(cur_chain_dim);
                   [~,~,V] = svd(obj.Jac_rep{cur_chain,index_map},'econ');
                   N_app = (I-V*alpha_diag*V');
                   % TODO 
                   % add the possibility of decide the order of the
                   % repellers
                   %---
                   N = N*N_app;
                end
            otherwise
                error('Unexpected structure for repulsor');
        end
            
      end
      
      
      % ALL THE FUNCTION BELOW WORK ONLY WITH cartesian_x REPELLERS 
      % in this function im building the extended jcobian of the repellers
      % I have to build the chain of jacobian repellers too
      function SetJacob(obj,cur_rob,q,qd,chain,task)
           
         switch obj.type_of_rep_strct{chain}

             % in this case i obtain a huge jacobian by stacking the single jacobian one over another     
             case 'extended'
                 [J,~,x]=obj.DirKin(cur_rob,q,qd,chain,task);
                 if(task==1)
                      obj.Jac_rep{chain}(1:obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task);
                 else
                      obj.Jac_rep{chain}( (obj.task_dimension(chain,task - 1) + 1) : obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task); 
                 end
             case 'stacked' 
                 obj.Jac_rep{chain}{task} = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task);
             otherwise
                 error('Unexpected structure for repulsor');
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