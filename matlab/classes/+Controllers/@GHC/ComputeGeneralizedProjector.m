function projector_list = ComputeGeneralizedProjector(obj,ind_subchain,J_list,t)
    
    n_task=obj.subchains.GetNumTasks(ind_subchain);
    I = eye(obj.subchains.GetNumLinks(ind_subchain));
    projector_list = cell(n_task,1);
    app_alpha=zeros(n_task,1);
    %size_vector = zeros(n_task,1);
    Js = [];
    alpha_vec_s_diag = [];
    
    %DEBUG
    t
    %--- 
    index_alpha = 1;
    for j = 1 : n_task
        
        % read and store the current value of alpha functions for each task 
        for k = 1:n_task
            app_alpha(k) = obj.alpha{ind_subchain,index_alpha}.GetValue(t);
            index_alpha = index_alpha + 1;
        end
        % sort vector 
        [alpha_vec_s,i]=sort(app_alpha,'descend');
        
        % build alpha_s and Js
        for k = 1:n_task
            J_app = J_list{i(k)};
            %DEBUG
            %nsv = rank(J_app);
            %---
            %size_vector(k) = size(J_app,1);
            Js = [Js;J_app];
            % build diagonal of the matrix a_s in form of vector.
            alpha_vec_s_diag = [alpha_vec_s_diag; alpha_vec_s(k)*ones(size(J_app,1),1)];
        end
        
        %DEBUG
%         nsv = rank(Js);
        %---
        
        % compute the origin 
        [B,origin,r]=GetOrthBasis(Js,obj.epsilon);
        %DEBUG
%          B'*B
        %---
        % take the element in alpha_vec_s_diag that appear after the
        % orthonormalization
        for k = 1:r
            alpha_vec_s_diag_origin(k) = alpha_vec_s_diag(origin(k));
        end
        %DEBUG
%         origin
%         test_mat=diag(alpha_vec_s_diag_origin)
        %---
        % compute projector 
        projector_list{j} = I - B'*diag(alpha_vec_s_diag_origin)*B; 
        alpha_vec_s_diag = [];
        Js=[];
    end




end



function [B,origin,r]=GetOrthBasis(Js,epsilon)
    % nbcol corresponds to the number of DOF of the kinematic chain
    nbrow = size(Js,1);
    nbcol = size(Js,2);
    B = zeros(nbrow,nbcol);
    i = 1;
    
    for k = 1 : nbrow
        
        B(i,:) = Js(k,:);
        
        for j=1:i-1
            B(i,:) = B(i,:) - ( B(i,:)*B(j,:)' ) * B(j,:);
        end
        
        if(norm(B(i,:))>epsilon)
            B(i,:) = B(i,:)/norm(B(i,:));
            origin(i) = k;
            i = i + 1;    
        end
        
        if i > nbcol
            break      
        end
        
    end

    r=i-1;
    B=B(1:r,:);

end