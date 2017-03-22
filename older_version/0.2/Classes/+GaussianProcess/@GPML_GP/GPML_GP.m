classdef GPML_GP < GaussianProcess.AbstractGP
    
   properties
      X
      Y
      Y_original
      upper_bound
      lower_bound
      likelihood
      inference
      cov 
      mean
      hyp0
      hyp
      Ncg
      normalization % activate normalization
   end
       
    
   methods
       %% TODO for now I setup everything inside the constructor then I will
       % do it in a flexible way
       function obj = GPML_GP()
           obj.likelihood = @likGauss;
           sn = 0.2;
           obj.hyp0.lik  = log(sn);
           obj.inference = @infGaussLik;
           obj.cov = {@covSEiso}; 
           ell = 1/4; sf = 1;                       
           obj.hyp0.cov  = ([ell;sf]);
           obj.mean = [];
           %obj.hyp0.mean = [];
           obj.Ncg = 100;
           pbj.normalization = false;
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
           
           if obj.Ncg == 0
             obj.hyp = obj.hyp0;
           else
             not_optimized = true;  
             hyp0_try = obj.hyp0;
             while(not_optimized)  
                 [obj.hyp, Fx, iter]  = minimize(hyp0_try,'gp', -obj.Ncg, obj.inference, obj.mean, obj.cov, obj.likelihood, obj.X, obj.Y); % opt hypers
                 if(prod(hyp0_try.cov == obj.hyp.cov))
                    hyp0_try.lik = log( (10*ones(size(hyp0_try.lik)) - 0.001*ones(size(hyp0_try.lik)) ).*rand(size(hyp0_try.lik))) + 0.001*ones(size(hyp0_try.lik));
                    hyp0_try.cov = (30*ones(size(hyp0_try.cov))).*rand(size(hyp0_try.cov));
                 else
                     obj.hyp0 = hyp0_try;
                     not_optimized = false;
                 end
             end
           end
       end
       
       %% TODO understand if it is usefull
       function Train(obj)
           
       end
       
       function Update(obj,x_n,y_n,train) 
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
           
           if obj.Ncg == 0
               obj.hyp = obj.hyp0;
           else          
                         %minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, x, y);
                % we need to minize respect of the initial obj.hyp0 to manage the variation in bounds       
               obj.hyp = minimize(obj.hyp0,@gp, -obj.Ncg, obj.inference, obj.mean, obj.cov, obj.likelihood, obj.X, obj.Y); % opt hypers
           end
       end
       
       function [ymu,ys2] = Predict(obj,x_t)
                                   %[m s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, x);
           [ymu, ys2, fmu, fs2 ] = gp(obj.hyp, obj.inference, obj.mean, obj.cov, obj.likelihood, obj.X, obj.Y, x_t); 
       end
       
       
       function [out]=Normalize(obj,Y)
           if(obj.lower_bound ~= obj.upper_bound)
               min_x = repmat(obj.lower_bound,length(Y),1);
               out=(Y-min_x)/(obj.upper_bound - obj.lower_bound); 
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
       
       
       
       function plot3D(obj,bounds,step)
           x_t=bounds(1,1):step:bounds(2,1);
           y_t=bounds(1,2):step:bounds(2,2);
           
           [X1_t,X2_t]=meshgrid(x_t,y_t);
           
           x1_t_vec=X1_t(:);
           x2_t_vec=X2_t(:);

           x_tt = [x1_t_vec,x2_t_vec];
           [m,s2] = obj.Predict(x_tt);
%           

%                 covfunc = @covSEiso; 
%                 likfunc = @likGauss; 
%                 hyp2.cov = log([10;20]);    
%                 hyp2.lik = log(0.1);
%                 hyp2 = minimize(hyp2, @gp, -100, @infExact, [], covfunc, likfunc, obj.X, obj.Y);
%                 %exp(hyp2.lik)
%                 %nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y);
%                 [m s2] = gp(hyp2, @infExact, [], covfunc, likfunc, obj.X, obj.Y, x_tt);
            
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