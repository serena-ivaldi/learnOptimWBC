%% i create this class as a backup for the old particle class and to ensure 
%% compatilibilty between this class and the crossEntropy (an attempt to merge the CrossEntropy mixture of gaussian method with the (1+1)CMA-ES) 


classdef ParticleExperimental < handle
   
   
   
   properties
      % external parameters (visible from mixture of gaussian)-------------
      mean;    % mean vector 
      C;       % covariance matrix  
      % internal parameters -----------------------------------------------
      n;       % dimension of parameter space
      sigma;   % exploration rate
      A;       % cholescky factor of the covariance matrix
      V;
      v;
      s;
      d;
      c;
      c_p;
      P_succ;
      P_target;
      c_cov_plus;
      c_cov_minus;
      c_c;
      beta;     
      performances;
      active; % this is a flag that determines if the particle is active so i have to use the internal evolution agorithm of the particle
      % for visualization (2d only)
      X_mesh;
      Y_mesh;
   end
            
   methods
      
      function obj = ParticleExperimental(size_action,maxAction,minAction,n_constraints,nIterations,explorationRate)
         obj.n = size_action;
         obj.mean = zeros(nIterations, obj.n);
         obj.sigma = zeros(nIterations,1);
         % initialize sigma
         obj.sigma(1) = explorationRate;
         obj.C{1} = diag((maxAction - minAction)/2);
         obj.A{1} = chol(obj.C{1});  
         for j = 1 : n_constraints
            obj.v(j,:) = zeros(1,obj.n);    
         end
         obj.V{1} = obj.v;   %cell vector of matrix where each v is row
         obj.s = zeros(nIterations,obj.n); 
         obj.d = 1 + obj.n/2;
         obj.c =  2/ (obj.n + 2);
         obj.c_p = 1/12;
         obj.P_succ = 0;   % im not sure
         obj.P_target = 2/12;
         obj.c_cov_plus = 2 /(obj.n^(2) + 6);
         obj.c_cov_minus = 0.4/(obj.n^(1.6) + 1);
         obj.c_c = 1/(obj.n + 2);
         obj.beta = 0.1/(obj.n + 2);
         obj.performances = zeros(nIterations,1);
         obj.active = false;
         
         x_mesh = minAction(1):.1:maxAction(1); %// x axis
         y_mesh = minAction(2):.1:maxAction(2); %// y axis
         [obj.X_mesh,obj.Y_mesh] = meshgrid(x_mesh,y_mesh); %// all combinations of x, y
      end
   
      function Evolve(obj,constraints_active,constraints,k,z,offsprings,performances_new)
          if(constraints_active)
            violated_constrained = find(constraints);
          end
          if(~isempty(violated_constrained)) % some constraints are violated
             obj.v = obj.V{k};
             for j = violated_constrained
               obj.v(j,:) = (1-obj.c_c)*obj.V{k}(j,:) + obj.c_c*(obj.A{k}*z')';                                           %only if the constraints is violated udpate exponentially fading record vj 
             end
             obj.V{k+1} = obj.v;
             index = 1;obj.v(j,:)  % to fix no commit without fixing
             for j = violated_constrained
               w(index,:) = (obj.A{k}^(-1)*obj.v(j,:)')';
               index = index + 1;
             end
             obj.mean(k+1,:) = obj.mean(k,:);                                                                                                                             % no update mean
             obj.s(k+1,:) = obj.s(k,:);                                                                                                                                      % no update s
             value = zeros(size(obj.A{1}));
             index = 1;
             for j = violated_constrained
               value = value +  (obj.v(j,:)'*w(index,:))/(w(index,:)*w(index,:)');
               index = index + 1;
             end
             obj.A{k+1} = obj.A{k} - (obj.beta)/length(violated_constrained) * value;                                                             % update A if constraint violation is true
             obj.sigma(k+1) = obj.sigma(k);                                                                                                                              % no update sigma
          else % all the constraints are satisfacted
             if(performances_new > obj.performances(k))
                obj.P_succ = (1-obj.c_p)*obj.P_succ + obj.c_p; 
             else
                obj.P_succ = (1-obj.c_p)*obj.P_succ; 
             end
             obj.sigma(k+1) = obj.sigma(k)*exp( (1/obj.d) * (obj.P_succ - obj.P_target) / (1-obj.P_target) );                                                                                % update sigma
             if(performances_new > obj.performances(k)) % perfomance is better
                obj.mean(k+1,:) = offsprings;                                                                                                                             % update mean
                obj.performances(k+1) = performances_new;
                obj.s(k+1,:) = (1-obj.c)*obj.s(k,:) + sqrt(obj.c*(2-obj.c))*(obj.A{k}*z')';                                       %only if the constraints are not violated udpate exponentially fading record s
                w = (obj.A{k}^(-1)*obj.s(k+1,:)')'; 
                obj.A{k+1} = sqrt(1 - obj.c_cov_plus)*obj.A{k} + ( sqrt(1-obj.c_cov_plus)/norm(w)^2 )*(sqrt(1 + (obj.c_cov_plus*norm(w)^2)/(1-obj.c_cov_plus) ) - 1 )*obj.s(k+1,:)'*w; % update A if perfor_new > perf(k)
                obj.V{k+1} = obj.V{k};                                                                                                                                       % no update v
             else % perfomance is worse
                obj.mean(k+1,:) = obj.mean(k,:);                                                                                                                            % no update mean
                obj.performances(k+1) = obj.performances(k);                                                                                                               
                obj.s(k+1,:) = obj.s(k,:);                                                                                                                                    % no update s
                obj.V{k+1} = obj.V{k};                                                                                                                                        % no update v
                if(k>5)   
                   if(performances_new > obj.performances(k-5)) % perfomance is worse but better than the last fifth predecessor
                      obj.A{k+1} = sqrt(1 + obj.c_cov_minus)*obj.A{k} + ( sqrt(1 + obj.c_cov_minus)/norm(z)^2 )*( sqrt(1 - (obj.c_cov_minus*norm(z)^2)/(1 + obj.c_cov_minus)) - 1 )*obj.A{k}*(z'*z); % update A if perf_new>perf(k-5)
                   else
                      obj.A{k+1} = obj.A{k};                                                                                                                                     % A no update
                   end
                else
                   obj.A{k+1} = obj.A{k};         
                end
             end
          end
      end
      function SetMean(obj,mean,index)
         obj.mean(index,:) = mean;
      end
      function SetCov(obj,cov,index)
         %DEBUG 
         %cov
         %--
         obj.C{index} = cov;
         obj.A{index} = chol(obj.C{index}); 
      end
      function mean = GetMean(obj,index)
         mean = obj.mean(index,:);
      end
      function cov = GetCov(obj,index)
         cov=obj.C{index};
      end
      function Sample()
      end
      
      function Plot(obj,iter)
        Z = mvnpdf([obj.X_mesh(:) obj.Y_mesh(:)],obj.GetMean(iter),obj.GetCov(iter)); %// compute Gaussian pdf
        Z = reshape(Z,size(obj.X_mesh)); %// put into same size as X, Y
        contour(obj.X_mesh,obj.Y_mesh,Z), axis equal, hold on  %// contour plot; set same scale for x and y...
        %surf(X,Y,Z) %// ... or 3D plot
      end
  
   end
   
      
end