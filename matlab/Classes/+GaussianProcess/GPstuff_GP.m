classdef GPstuff_GP < GaussianProcess.AbstractGP
    
   properties
      X
      Y
      Y_original
      upper_bound   % they are bound defined on the value of y (the output)
      lower_bound   % they are bound defined on the value of y (the output)
      gp
      normalization % activate normalization
      zooming       % zooming effect
      zoom_center   % zooming around
      zoom_radius   % zooming radius     
      reduced_gp    % reduced_gp for the current zooming
   end
       
    
   methods
       %% TODO for now I setup everything inside the constructor then I will
       %% do it in a flexible way
       function obj = GPstuff_GP()
           %% to change at some point with matern function
           % construct GP 
           %cfse = gpcf_sexp('lengthScale',1,'magnSigma2',1,'magnSigma2_prior',prior_sqrtt('s2',10^2));
           cfse = gpcf_matern52('lengthScale',1,'magnSigma2',1);
           lik = lik_gaussian('sigma2', 0.001, 'sigma2_prior', prior_fixed);
           %% i try to raise the value of the jitter to avoid the problem of negative eigenvalue in the gramiam matrix
           obj.gp = gp_set('cf', {cfse}, 'lik', lik,'jitterSigma2',1e-6);
           obj.normalization = false;
       end
       
       function Init(obj,X_i,Y_i)
           obj.X = X_i;
           % normalization
           if(obj.normalization)
               obj.Y_original = Y_i;
               % output normalization
               obj.upper_bound = max(Y_i);
               obj.lower_bound = min(Y_i);
               Y_i_norm = obj.Normalize(obj.Y_original);
               obj.Y = Y_i_norm;
           else
               obj.Y = Y_i;
           end
           try
               % optimize hyper parameters
               options_opt = struct('Algorithm','interior-point','LargeScale','off','GradObj','on');
               obj.gp = gp_optim(obj.gp,obj.X,obj.Y,'optimf',@fmincon,'opt',options_opt);
               %obj.gp = gp_optim(obj.gp,obj.X,obj.Y);
           catch
                disp('optim falied due to non positve defined gramiam matrix')
           end
       end
       
       %% TODO understand if it is usefull
       function Train(obj,x_i,y_i)
           
       end
       
       function Update(obj,x_n,y_n) 
           obj.X(end + 1,:) = x_n;
           % normalization
           if(obj.normalization)
               obj.Y_original(end + 1) = y_n;
               update = obj.CheckBound(y_n);
               if(update)
                   % if i update the bound i need to renormalize all the point
                   % with the new bound
                   Y_i_norm = obj.Normalize(obj.Y_original);
                   obj.Y = Y_i_norm;
               else
                   % if i do not update the bound i just apply the old bound to
                   % the new one
                   y_n_norm = obj.Normalize(y_n);
                   obj.Y(end + 1) = y_n_norm;  
               end    
           else
               obj.Y(end + 1) = y_n;
           end
           %% TODEBUG i try to pass a normalize Y only for optimization to see if it makes work the gp better
           % optimize hyper parameters
           try
               options_opt = struct('Algorithm','interior-point','LargeScale','off','GradObj','on');
               obj.gp = gp_optim(obj.gp,obj.X,obj.Y,'optimf',@fmincon,'opt',options_opt);
               %obj.gp = gp_optim(obj.gp,obj.X,obj.Y);
           catch
               disp('optim falied due to non positve defined gramiam matrix')
               %% TODO introduce a function that restrict the data matrix in order to obtain a kernel matrix positive defined or maybe i can try to restart the kernel parameters
               %% this is the way to manage the kernel matrix
           end
       end
       
       function [ymu,ys2] = Predict(obj,x_t)
           % too slow
           %[ymu, ys2] = gp_pred(obj.gp, obj.X, obj.Y, x_t);
           % faster method working only with gaussian exact 
           %% zooming procedure for prediction
           if(obj.zooming)    
               [ymu, ys2] = obj.reduced_gp.Predict(x_t);
           %% standard prediction               
           else  
                 %% TOFIX this way to compute the prediction was giving me solutions so differents from the one 
                 %% that are computed with gp_pred() function and noy just that. The value that i found they where unreliable in the sense
                 %% that even a region of space with a lot of point was giving me huge variance or point and it was generating huge mistake in the mean computation too.
                 %% it is necessary to extrapolate from gp_pred only the working part and remove the rest of the function that is the real reason why this function is soo slow
%                [K, C] = gp_trcov(obj.gp,obj.X);
%                %% doing this modification change the behaviour of the prediction a bit
%                %invC = inv(C);
%                invC = C \ eye(size(C,1)); 
%                %a = C\obj.Y;
%                a = invC * obj.Y;
%                %%
%                Knx = gp_cov(obj.gp,x_t,obj.X);
%                Kn = gp_trvar(obj.gp,x_t);
%                % mean
%                ymu = Knx*a; ymu=ymu(1:size(x_t,1));
%                invCKnxt = invC*Knx';
%                % variance
%                ys2 = Kn - sum(Knx.*invCKnxt',2); 
%                % i need to check for negative variance because it could happens
%                % and the previous approach is less robust to numerical error
%                % i check if the variance are correct than 
%                index = ys2 > 0;
%                if(~prod(index))
                    %% if i obtain a negative variance i want to use the GP_stuff that is slower but more robust
                    %%  TODO extract only the working part from gp_pred
                    %disp('negative variance')
                    [ymu, ys2] = gp_pred(obj.gp, obj.X, obj.Y, x_t);
%               end
           end
       end
       
       function ActivateZooming(obj,zoom_center,zoom_radius)
           obj.zooming = true;       % zooming effect
           if(iscolumn(zoom_center))
               zoom_center = zoom_center';
           end
           obj.zoom_center = zoom_center; % zooming around
           obj.zoom_radius = zoom_radius;  % zooming radius
           [X_zoom,Y_zoom] = obj.ReducedDataset();
           obj.reduced_gp = GaussianProcess.GPstuff_GP();
           obj.reduced_gp.Init(X_zoom,Y_zoom);
       end
       
       function DeactivateZooming(obj)
           obj.zooming = false;   % zooming effect
           obj.zoom_center = []; % zooming around
           obj.zoom_radius = [];  % zooming radius
           obj.reduced_gp  = [];
       end
       
       function [X_zoom,Y_zoom] = ReducedDataset(obj)
           %% TODO fix this to manage the case with only one point in the dataset X
           len = length(obj.X);
           extend_zoom_center = repmat(obj.zoom_center,len,1);
           extend_zoom_radius = repmat(obj.zoom_radius,len,1);
           dist = sqrt(sum((obj.X - extend_zoom_center).^2,2));
           ind = dist <= extend_zoom_radius;
           X_zoom = obj.X(ind,:);
           Y_zoom = obj.Y(ind);      
       end
       
       function [out]=Normalize(obj,Y)
           if(obj.lower_bound ~= obj.upper_bound)
               min_y = repmat(obj.lower_bound,length(Y),1);
               out=(Y-min_y)/(obj.upper_bound - obj.lower_bound); 
           else
               out = Y;
           end
       end
       
       function update = CheckBound(obj,y)
           update = false;
           if(y>obj.upper_bound)
               obj.upper_bound = y;
               update = true;
           elseif(y<obj.lower_bound)
               obj.lower_bound = y;
               update = true;
           end
       end
       %% TODO at some point use this function to print the gaussian inside 
       function plot3D(obj,bounds,step)
           x_t=bounds(1,1):step:bounds(2,1);
           y_t=bounds(1,2):step:bounds(2,2);
           
           [X1_t,X2_t]=meshgrid(x_t,y_t);
           
           x1_t_vec=X1_t(:);
           x2_t_vec=X2_t(:);

           x_tt = [x1_t_vec,x2_t_vec];
           [m,s2] = obj.Predict(x_tt);
            
           m_mat  =vec2mat(m,size(X1_t,1)); 
           % figure of fitness value
           figure; hold on;
           scatter3(obj.X(:,1),obj.X(:,2),obj.Y);
           surf(X1_t,X2_t,m_mat');
           % figure of variance
           s2_mat = vec2mat(s2,size(X1_t,1));
           figure; hold on;
           scatter3(obj.X(:,1),obj.X(:,2),zeros(size(obj.Y)));
           %surf(X1_t,X2_t,s2_mat');  
           contourf(X1_t,X2_t,s2_mat');
       end
            
   
   end
   
  
    
end