classdef Particle < handle
   
   
   % the idea here is that  i can create a particle at every moments during the
   % execution of the algorithm and the particles can be updated even in
   % different iterations. so i need a way to give the particle de
   % capabalities to know which is their interior iteration --->
   % current_index;
   
   properties
      %maxAction
      %minAction
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
      % for visualization (2d only)
      X_mesh;
      Y_mesh;
      current_index; % this index tell us where we are inside the current 
   end
            
   methods
      
      function obj = Particle(size_action,maxAction,minAction,n_constraints,nIterations,explorationRate)
         %obj.maxAction = maxAction;
         %obj.minAction = minAction;
         obj.n = size_action;
         obj.mean = zeros(nIterations, obj.n);                                             % mean vector 
         obj.sigma = zeros(nIterations,1);                                                 % sigma vector
         obj.C     = cell(nIterations,1);                                                  % C vector 
         obj.A     = cell(nIterations,1);                                                  % A vector 
         obj.V     = cell(nIterations,1);                                                  % V vector
         obj.performances = zeros(nIterations,1);                                          % perfomances vector
         obj.s = zeros(nIterations,obj.n);                                                 % s vector
         % initialize sigma
         obj.sigma(1) = explorationRate;                        
         obj.C{1} = diag((maxAction - minAction)/2);
         obj.A{1} = chol(obj.C{1});  
         for j = 1 : n_constraints
            obj.v(j,:) = zeros(1,obj.n);    
         end
         obj.V{1} = obj.v;   %cell vector of matrix where each v is row
         obj.d = 1 + obj.n/2;
         obj.c =  2/ (obj.n + 2);
         obj.c_p = 1/12;
         obj.P_succ = 0;   % im not sure
         obj.P_target = 2/12;
         obj.c_cov_plus = 2 /(obj.n^(2) + 6);
         obj.c_cov_minus = 0.4/(obj.n^(1.6) + 1);
         obj.c_c = 1/(obj.n + 2);
         obj.beta = 0.1/(obj.n + 2); 
         x_mesh = minAction(1):.1:maxAction(1); %// x axis
         y_mesh = minAction(2):.1:maxAction(2); %// y axis
         [obj.X_mesh,obj.Y_mesh] = meshgrid(x_mesh,y_mesh); %// all combinations of x, y
         obj.current_index = 1;
      end
   
%       function [candidate] = Sample(obj)
%          candidate = obj.GetMean(index) + mvnrnd( zeros(1, obj.n),obj.GetCov(index));
%          % saturation
%          candidate(1, candidate(1,:) > obj.maxAction) = obj.maxAction(candidate(1,:) > obj.maxAction);
%          candidate(1, candidate(1,:) < obj.minAction) = obj.minAction(candidate(1,:) < obj.minAction);
%       end
      
      function Evolve(obj,constraints_active,constraints,z,offsprings,performances_new)
          if(constraints_active)
            violated_constrained = find(constraints);
          end
          if(~isempty(violated_constrained)) % some constraints are violated
             obj.v = obj.V{obj.current_index};
             for j = violated_constrained
               obj.v(j,:) = (1-obj.c_c)*obj.V{obj.current_index}(j,:) + obj.c_c*(obj.A{obj.current_index}*z')';                                           %only if the constraints is violated udpate exponentially fading record vj 
             end
             obj.V{obj.current_index+1} = obj.v;
             index = 1;obj.v(j,:)  % to fix no commit without fixing
             for j = violated_constrained
               w(index,:) = (obj.A{obj.current_index}^(-1)*obj.v(j,:)')';
               index = index + 1;
             end
             obj.mean(obj.current_index + 1,:) = obj.mean(k,:);                                                                                                                             % no update mean
             obj.s(obj.current_index + 1,:) = obj.s(k,:);                                                                                                                                      % no update s
             value = zeros(size(obj.A{1}));
             index = 1;
             for j = violated_constrained
               value = value +  (obj.v(j,:)'*w(index,:))/(w(index,:)*w(index,:)');
               index = index + 1;
             end
             obj.A{obj.current_index + 1} = obj.A{obj.current_index} - (obj.beta)/length(violated_constrained) * value;                                                             % update A if constraint violation is true
             obj.sigma(obj.current_index + 1) = obj.sigma(obj.current_index);                                                                                                                              % no update sigma
          else % all the constraints are satisfacted
             if(performances_new > obj.performances(obj.current_index))
                obj.P_succ = (1-obj.c_p)*obj.P_succ + obj.c_p; 
             else
                obj.P_succ = (1-obj.c_p)*obj.P_succ; 
             end
             obj.sigma(obj.current_index+1) = obj.sigma(obj.current_index)*exp( (1/obj.d) * (obj.P_succ - obj.P_target) / (1-obj.P_target) );                                                                                % update sigma
             if(performances_new > obj.performances(obj.current_index)) % perfomance is better
                obj.mean(obj.current_index + 1,:) = offsprings;                                                                                                                             % update mean
                obj.performances(obj.current_index + 1) = performances_new;
                obj.s(obj.current_index + 1,:) = (1-obj.c)*obj.s(obj.current_index,:) + sqrt(obj.c*(2-obj.c))*(obj.A{obj.current_index}*z')';                                       %only if the constraints are not violated udpate exponentially fading record s
                w = (obj.A{obj.current_index}^(-1)*obj.s(obj.current_index + 1,:)')'; 
                obj.A{obj.current_index + 1} = sqrt(1 - obj.c_cov_plus)*obj.A{obj.current_index} + ( sqrt(1-obj.c_cov_plus)/norm(w)^2 )*(sqrt(1 + (obj.c_cov_plus*norm(w)^2)/(1-obj.c_cov_plus) ) - 1 )*obj.s(obj.current_index+1,:)'*w; % update A if perfor_new > perf(k)
                obj.V{obj.current_index + 1} = obj.V{obj.current_index};                                                                                                                                       % no update v
             else % perfomance is worse
                obj.mean(obj.current_index + 1,:) = obj.mean(obj.current_index,:);                                                                                                                            % no update mean
                obj.performances(obj.current_index + 1) = obj.performances(obj.current_index);                                                                                                               
                obj.s(obj.current_index + 1,:) = obj.s(obj.current_index,:);                                                                                                                                    % no update s
                obj.V{obj.current_index + 1} = obj.V{obj.current_index};                                                                                                                                        % no update v
                if(obj.current_index > 5)   
                   if(performances_new > obj.performances(obj.current_index - 5)) % perfomance is worse but better than the last fifth predecessor
                      obj.A{obj.current_index + 1} = sqrt(1 + obj.c_cov_minus)*obj.A{ obj.current_index } + ( sqrt(1 + obj.c_cov_minus)/norm(z)^2 )*( sqrt(1 - (obj.c_cov_minus*norm(z)^2)/(1 + obj.c_cov_minus)) - 1 )*obj.A{obj.current_index}*(z'*z); % update A if perf_new>perf(k-5)
                   else
                      obj.A{obj.current_index + 1} = obj.A{obj.current_index};                                                                                                                                     % A no update
                   end
                else
                   obj.A{obj.current_index + 1} = obj.A{obj.current_index};         
                end
             end
             % move current_index one step forward 
             obj.current_index = obj.current_index + 1;
          end
      end
      
      %% set and get work by working on the last value according to current_index
      function SetMean(obj,mean)
         obj.mean(obj.current_index,:) = mean;
      end
      function SetCov(obj,cov)
         %DEBUG 
         %cov
         %--
         obj.C{obj.current_index} = cov;
         obj.A{obj.current_index} = chol(obj.C{obj.current_index}); 
      end
      function mean = GetMean(obj)
         mean = obj.mean(obj.current_index,:);
      end
      function cov = GetCov(obj)
         cov=obj.C{obj.current_index};
      end
      
      function A = GetCholCov(obj)
         A=obj.A{obj.current_index};
      end
      
      function sigma = GetSigma(obj)
          sigma = obj.sigma{obj.current_index};
      end
      function y = GetBestPerfomance(obj)
          y = obj.performances(obj.current_index);
      end
      
      function Plot(obj)
        Z = mvnpdf([obj.X_mesh(:) obj.Y_mesh(:)],obj.GetMean(),obj.GetCov()); %// compute Gaussian pdf
        Z = reshape(Z,size(obj.X_mesh)); %// put into same size as X, Y
        contour(obj.X_mesh,obj.Y_mesh,Z), axis equal, hold on  %// contour plot; set same scale for x and y...
        %surf(X,Y,Z) %// ... or 3D plot
      end
  
   end
   
      
end