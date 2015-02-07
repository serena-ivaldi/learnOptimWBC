
classdef  Repellers < handle
    
   properties
      chain_dof            % number of degrees of freeedom for each kinematic chain 
      target_link;         % vector that define wich kind of link interact with the repellers,one for every kinematic chain
      type;                % cartesian_x,cartesian_rpy, joint vector  
      mask;                % vector of vector(3) (col vec) that contains a mask that specify what i want to control for the specific task. for example x and z (control a subset of variable) mask = (1;0:1)
      J_rep_func           % this specify wich kind of function i want to use to compute the repellers jacobian
      obstacle_ref         % vector of index that specify for each repellers which is the obstacle  G_OB (global vector of Obstacle) associated to the current repeller  
      task_dimension       % vector that define the cumulative dimension for each repulsive task. i use this as a vector of pointer to build the jacobian. 
      single_alpha         % is a vector of vector of flag 0 and 1 that define for each repellers if i want one alpha for each dimension of my repellers or only one alpham.If single_alpha = 1 use only one alpha for the whole controller
      J_damp               % is the value used to perform adamped least square inversion of my extended jacobian in the 'extended_decoupled' case
      type_of_rep_strct    % with this parameter i can specify wich kind of repellers structure i want to use : 'extended_combine' or 'stacked' (column vector of string)  one for each kinematic chain
      n_alpha_x_chain      % column vector that on the basis of the single_alpha contain the number of "different" alpha for repellers per kinematic chain 
      map_from_alpha_to_rep% this vector has the dimension of the number of alpha but say to me at wich repellors the alpha are reffered so is basically a vector of pointers, one for each chain 
      repellers_fun;       % cell array of function of repellers of type (i,j) where j is the task of the i-th kinematic chain
      Jac_rep              % cell array of the final jacobian is the final jacobian of the repellers
                    
   end
       
    
   methods
      %type_of_J_rep specify the kind of repellers that i want to use
      %chain_dof is a vector with the dimension 
      function obj = Repellers(chain_dof,target_link,type,mask,type_of_J_rep,obstacle_ref,single_alpha,J_damp,type_of_rep_strct) 
          
         obj.target_link = target_link;
         
       
         if(getnameidx({'joint' 'cartesian_x' 'cartesian_rpy'} , type) ~= 0 )
            obj.type = type;
         end
         obj.mask = mask;
         obj.obstacle_ref = obstacle_ref;
         obj.chain_dof = chain_dof;
         obj.J_damp = J_damp;
         % in this cycle i copy the value that i have in RuntimeVariable
         % inside the object in such a way that i dont care about the
         % structure of  single_alpha in RuntimeVariable. the only things that
         %  is necesessary is that the single_alpha is sufficiently long.    
         for i=1:size(mask,1)
             for j=1:size(mask(i,:),2) 
                obj.single_alpha{i}(1,j) = single_alpha{i}(1,j);
             end
         end
         
         obj.type_of_rep_strct = type_of_rep_strct;
         
         % when i move on the row i cheching for diffrent kinematic chain
         for i=1:size(mask,1)
             switch obj.type_of_rep_strct{i}
                 case 'extended_combine' 
                     % in this cycle i pre allocate the jacobian for each chain and 
                     % i compute from mask the dimension of each task
                     dim = 0;
                         % im looking for all the repulsor that correspond to
                         % the number of mask on a row 
                         for j=1:size(mask(i,:),2)
                             dim = dim + nnz(mask{i,j});  
                             obj.task_dimension(i,j) = dim;
                         end
                     obj.Jac_rep{i} = zeros(dim,chain_dof(i));
                 case 'stacked'
                     % im looking for all the repulsor that correspond to
                     % the number of mask on a row 
                     for j=1:size(mask(i,:),2)
                            obj.Jac_rep{i,j} = zeros(nnz(obj.mask{i,j}),chain_dof(i));
                     end
                 case 'extended_decoupled'
                      for j=1:size(mask(i,:),2)
                            obj.Jac_rep{i,j} = zeros(nnz(obj.mask{i,j}),chain_dof(i));
                      end
                      % because of in this branch i have only one alpha for
                      % each repellor i change the value of single_alpha to
                      % assure that this condition is met
                      for ii=1:size(mask,1)
                         for jj=1:size(mask(ii,:),2) 
                            obj.single_alpha{ii}(1,jj) = 1;
                         end
                     end
                 otherwise 
                error('Unexpected structure for repulsor');
             end  
         end
         
         % in this cycle i compute the number of alpha for each kinematic
         % chain for the repellers
         for i=1:size(mask,1)
            obj.n_alpha_x_chain(i)=obj.ComputeNumberOfWeightFuncRep(i);    
         end
         
         % in this cycle i build the vector of function handles to compute
         %  repellers jacobian
         for i=1:size(mask,1)
             for j=1:size(mask(i,:),2)
                obj.repellers_fun{i,j} = eval(strcat('@',type_of_J_rep{i,j}));
             end
             
         end
         
         
      end    
      
      
      % map_from_alpha_to_rep is a vector where each position correspond to
      % an alpha function and each value correspond to reference repellers 
      % for example  map_from_alpha_to_rep[1] = 1   and
      % map_from_alpha_to_rep[2] = 2 map_from_alpha_to_rep[3] = 2
      % map_from_alpha_to_rep[4] = 2
      % means that the first repellors has only one activation policy
      % instead the second repellors has 3 activation policy
      
      function n=ComputeNumberOfWeightFuncRep(obj,cur_chain)
           n = 0;
           index = 1;
           for j=1:size(obj.mask(cur_chain,:),2) 
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
      
      
      function N=ComputeProjector(obj,cur_chain,cur_chain_dim,task_per_cur_chain,alpha,t)       
        
        n_of_alpha_repellers=obj.GetNumberOfWeightFuncRep(cur_chain);  
          
        switch obj.type_of_rep_strct{cur_chain}

            case 'extended_combine' 
                
                alpha_vec=zeros(n_of_alpha_repellers,1);
                index_alpha_vec = 1;
                for index_map = 1:n_of_alpha_repellers;
                    current_repellor = obj.map_from_alpha_to_rep{cur_chain}(index_map);
                    if (obj.single_alpha{cur_chain}(current_repellor))
                        % i have to use a different index that maps the
                        % current number of alpha to the original repellers
                        for k=1:nnz(obj.mask{cur_chain,current_repellor})
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
                % inizialized matrix of repulsor
                N = eye(cur_chain_dim);
                index_map = 1;
                while index_map <= n_of_alpha_repellers
                    
                   current_repellor = obj.map_from_alpha_to_rep{cur_chain}(index_map); 
                   alpha_vec = zeros(1,nnz(obj.mask{cur_chain,current_repellor}));
                   if (obj.single_alpha{cur_chain}(current_repellor))
                        % i have to use a different index that maps the
                        % current number of alpha to the original repellers
                        for kk=1:nnz(obj.mask{cur_chain,current_repellor})
                            alpha_vec(kk)=alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t);   
                        end
                   else 
                        for k=1:nnz(obj.mask{cur_chain,current_repellor})
                            alpha_vec(k) = alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t);  
                            index_map = index_map + 1;
                        end       
                   end
                   alpha_diag = diag(alpha_vec);
                   I = eye(cur_chain_dim);
                   [~,~,V] = svd(obj.Jac_rep{cur_chain,current_repellor},'econ');
                   N_app = (I-V*alpha_diag*V');
                   % TODO 
                   % add the possibility of decide the order of the
                   % repellers
                   %---
                   N = N*N_app;
                   index_map = index_map + 1;
                end
            % in this branch i have only one activation policy for each repellors     
            case 'extended_decoupled'
                N = eye(cur_chain_dim);
                I = N;
                Jext = [];
                index_map = 1;
                while index_map <= n_of_alpha_repellers
                   Jext =  [Jext;alpha{cur_chain,task_per_cur_chain+index_map}.GetValue(t)*obj.Jac_rep{cur_chain,index_map}];
                   index_map = index_map + 1;
                end
                JJt=Jext*Jext';
                I_damp = obj.J_damp*eye(size(JJt,1));
                N = I - (Jext'/(I_damp + JJt))*Jext';
            otherwise
                error('Unexpected structure for repulsor');
        end
            
      end
      
      
      % ALL THE FUNCTION BELOW WORK ONLY WITH cartesian_x REPELLERS 
      % in this function im building the extended jcobian of the repellers
      % I have to build the chain of jacobian repellers too
      function SetJacob(obj,cur_rob,q,qd,chain,task)
           
         [J,~,x]=obj.DirKin(cur_rob,q,qd,chain,task);  
         
         % i use only one type of concantenation for each 
         switch obj.type_of_rep_strct{chain}
             % in this case i obtain a huge jacobian by stacking the single jacobian one over other     
             case 'extended_combine'
                 if(task==1)
                      obj.Jac_rep{chain}(1:obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task);
                 else
                      obj.Jac_rep{chain}( (obj.task_dimension(chain,task - 1) + 1) : obj.task_dimension(chain,task) , :) = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task); 
                 end
             case 'stacked' 
                 obj.Jac_rep{chain,task} = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task);
                 
             case 'extended_decoupled'
                  obj.Jac_rep{chain,task} = feval(obj.repellers_fun{chain,task},obj,x,J,chain,task); 
                 
             otherwise
                 error('Unexpected structure for repulsor');
         end 
      end
      
      
      function J_rep = DirectionCartesian(obj,direct_kin,J_old,chain,task)
        global G_OB;    
          if(strcmp(obj.type{chain,task},'cartesian_x')) 
            J = ReshapeJacobian(J_old,[],obj.chain_dof(chain),obj.target_link{chain}(1,task),obj.mask{chain,task},'trans');
          elseif(strcmp(obj.type{chain,task},'cartesian_rpy'))
            %J = ReshapeJacobian(J_old,[],obj.chain_dof(chain),obj.target_link(chain,task),obj.mask{chain,task},'rot');    
          end
          v = 2*(direct_kin - G_OB(obj.obstacle_ref(chain,task)).GetDescription()).^(2);
          J_rep = diag(v)*J;
          
      end
   end
   
   
   
    
end