classdef Particle < handle
   
   
   % the idea here is that  i can create a particle at every moments during the
   % execution of the algorithm and the particles can be updated even in
   % different iterations. so i need a way to give the particle de
   % capabalities to know which is their interior iteration --->
   % current_index;
   
   properties
      maxAction
      minAction
      % external parameters (visible from mixture of gaussian)-------------
      mean;    % mean vector 
      %C;       % covariance matrix  
      % internal parameters -----------------------------------------------
      n;       % dimension of parameter space
      sigma;   % exploration rate
      sigma_multiplier    % multiplier that define the region of appliance of the boost move
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
      status;        % active, candidate attained, merged
      clr;
      turns_of_inaction; % with turn_of_inaction i count i many turns the particle do not evolve in order to cancel it
      constraints; % this is a swtich that i sue to deactivate or activate constraints. Constraints is TRUE from default
   end
            
   methods
      
      function obj = Particle(size_action,maxAction,minAction,n_constraints,nIterations,explorationRate,start_candidate,start_perfomance,color)
         obj.maxAction = maxAction;
         obj.minAction = minAction;
         obj.n = size_action;
         obj.mean = zeros(nIterations, obj.n);                                             % mean vector 
         obj.sigma = zeros(nIterations,1);                                                 % sigma vector
         %obj.C     = cell(nIterations,1);                                                  % C vector 
         obj.A     = cell(nIterations,1);                                                  % A vector 
         obj.V     = cell(nIterations,1);                                                  % V vector
         obj.performances = zeros(nIterations,1);                                          % perfomances vector
         obj.s = zeros(nIterations,obj.n);                                                 % s vector
         % initiliaze mean 
         obj.mean(1, :) = start_candidate;
         %initialize perfomance 
         obj.performances(1) = start_perfomance;
         % initialize sigma
         obj.sigma(1) = explorationRate;           
         obj.sigma_multiplier = 1.5;
         C = diag((maxAction - minAction)/2);
         obj.A{1} = chol(C);  
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
         obj.status = 'active';
         obj.clr = color;
         obj.turns_of_inaction = 0;
         obj.constraints = true;
      end
   
%       function [candidate] = Sample(obj)
%          candidate = obj.GetMean(index) + mvnrnd( zeros(1, obj.n),obj.GetCov(index));
%          % saturation
%          candidate(1, candidate(1,:) > obj.maxAction) = obj.maxAction(candidate(1,:) > obj.maxAction);
%          candidate(1, candidate(1,:) < obj.minAction) = obj.minAction(candidate(1,:) < obj.minAction);
%       end
      
      function Evolve(obj,violated_constrained,z,offsprings,performances_new)
          %if(constraints_active)
          %  violated_constrained = find(constraints);
          %end
          if(~isempty(violated_constrained) && constraints) % some constraints are violated
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
             obj.mean(obj.current_index + 1,:) = obj.mean(obj.current_index,:);                                                                                                                             % no update mean
             obj.s(obj.current_index + 1,:) = obj.s(obj.current_index,:);                                                                                                                                      % no update s
             value = zeros(size(obj.A{1}));
             index = 1;
             for j = violated_constrained
               value = value +  (obj.v(j,:)'*w(index,:))/(w(index,:)*w(index,:)');
               index = index + 1;
             end
             obj.A{obj.current_index + 1} = obj.A{obj.current_index} - (obj.beta)/length(violated_constrained) * value;                                                             % update A if constraint violation is true
             obj.sigma(obj.current_index + 1) = obj.sigma(obj.current_index);                                                                                                                              % no update sigma
              % no update of mean turn of inaction + 1;
             obj.turns_of_inaction = obj.turns_of_inaction + 1;
          else % all the constraints are satisfacted
             if(performances_new > obj.performances(obj.current_index))
                obj.P_succ = (1-obj.c_p)*obj.P_succ + obj.c_p; 
             else
                obj.P_succ = (1-obj.c_p)*obj.P_succ; 
             end
             % sigma = sigma * exp(1/d * (P_succ - P_target)/(1 - P_target))
             obj.sigma(obj.current_index+1) = obj.sigma(obj.current_index)*exp( (1/obj.d) * (obj.P_succ - obj.P_target) / (1-obj.P_target) );                                                                                % update sigma
             if(performances_new > obj.performances(obj.current_index)) % perfomance is better
                obj.mean(obj.current_index + 1,:) = offsprings;                                                                                                                             % update mean
                obj.performances(obj.current_index + 1) = performances_new;
                % s = (1-c)s + sqrt(c*(2-c))*A*z
                obj.s(obj.current_index + 1,:) = (1-obj.c)*obj.s(obj.current_index,:) + sqrt(obj.c*(2-obj.c))*(obj.A{obj.current_index}*z')';                                       %only if the constraints are not violated udpate exponentially fading record s
                % w = A^(-1)*s
                w = (obj.A{obj.current_index}^(-1)*obj.s(obj.current_index + 1,:)')'; 
                % A = sqrt(1- c_cov_+)*A + sqrt(1 - c_cov_+)/norm(w)^2(sqrt(1 + (c_cov_+ * norm(w)^2)/(1- c_cov_+)) -1  )*s*w  --->   C = (1-c_cov_+)*C + c_cov_+*s*s^(t)
                obj.A{obj.current_index + 1} = sqrt(1 - obj.c_cov_plus)*obj.A{obj.current_index} + ( sqrt(1-obj.c_cov_plus)/norm(w)^2 )*(sqrt(1 + (obj.c_cov_plus*norm(w)^2)/(1-obj.c_cov_plus) ) - 1 )*obj.s(obj.current_index+1,:)'*w; % update A if perfor_new > perf(k)
                obj.V{obj.current_index + 1} = obj.V{obj.current_index};                                                                                                                                       % no update v
                % reset turn of inaction
                obj.turns_of_inaction = 0;
             else % perfomance is worse
                obj.mean(obj.current_index + 1,:) = obj.mean(obj.current_index,:);                                                                                                                            % no update mean
                obj.performances(obj.current_index + 1) = obj.performances(obj.current_index);                                                                                                               
                obj.s(obj.current_index + 1,:) = obj.s(obj.current_index,:);                                                                                                                                    % no update s
                obj.V{obj.current_index + 1} = obj.V{obj.current_index};                                                                                                                                        % no update v
                if(obj.current_index > 3)   
                   if(performances_new > obj.performances(obj.current_index - 3)) % perfomance is worse but better than the last fifth predecessor
                       % A = sqrt(1- c_cov_-)*A + sqrt(1 - c_cov_-)/norm(z)^2(sqrt(1 + (c_cov_- * norm(z)^2)/(1 + c_cov_-)) -1  )*A*z*z^(t)  --->   C = (1 + c_cov_-)*C - c_cov_-*(A*z)*(A*z)^(t)
                       obj.A{obj.current_index + 1} = sqrt(1 + obj.c_cov_minus)*obj.A{ obj.current_index } + ( sqrt(1 + obj.c_cov_minus)/norm(z)^2 )*( sqrt(1 - (obj.c_cov_minus*norm(z)^2)/(1 + obj.c_cov_minus)) - 1 )*obj.A{obj.current_index}*(z'*z); % update A if perf_new>perf(k-5)
                   else
                      obj.A{obj.current_index + 1} = obj.A{obj.current_index};                                                                                                                                     % A no update
                   end
                else
                   obj.A{obj.current_index + 1} = obj.A{obj.current_index};         
                end
                % no update of mean turn of inaction + 1;
                obj.turns_of_inaction = obj.turns_of_inaction + 1;
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
         A_cur=obj.A{obj.current_index};
         cov = A_cur'*A_cur;
      end
      
      function A = GetCholCov(obj)
         A=obj.A{obj.current_index};
      end
      
      function sigma = GetSigma(obj)
          sigma = obj.sigma(obj.current_index);
      end
      function y = GetBestPerfomance(obj)
          y = obj.performances(obj.current_index);
      end
      
      function DeactivateConstraints(obj)
          obj.constraints = false;
      end
      
      function ActivateConstraints(obj)
          obj.constraints = true;
      end
          
      
      function [mu,V_s,tlb,tup] = GetRotTraslBound(obj)
          p = 0.95; 
          mu = obj.GetMean()';
          C = obj.GetCov();
          %[~,D,V_s]=svd(C);
          [V_s, D] = eig(C);
          k = obj.conf2mahal(p, obj.n);
          L = k * sqrt(abs(diag(D)))*obj.GetSigma()*obj.sigma_multiplier;
          tlb = -L';
          tup = L';  
      end
      
      
       function ret = RotoTrasl(obj,x,mu,V_s)
           % to allow this function to work with multiple input i had to
           % rearrange a little bit the trasfromation 
           % basically im doing  ----> ret = mu + R*x   with  R = V_s
           mu = repmat(mu',size(x,1),1);
           ret = mu + x*V_s';
           % saturation 
           minaction = repmat(obj.minAction,size(ret,1),1);
           maxaction = repmat(obj.maxAction,size(ret,1),1);
           %ret(1, ret(1,:) > obj.maxAction) = obj.maxAction(ret(1,:) > obj.maxAction);
           %ret(1, ret(1,:) < obj.minAction) = obj.minAction(ret(1,:) < obj.minAction);
           ret(ret < minaction) = minaction(ret < minaction);
           ret(ret > maxaction) = maxaction(ret > maxaction);
        end
      %% visualization functions
      function Plot(obj)
          mu = obj.GetMean()';
          C = obj.GetCov();
          if size(C) ~= [2 2]
              disp('Sigma must be a 2 by 2 matrix'); 
          end
          if length(mu) ~= 2, 
              disp('mu must be a 2 by 1 vector'); 
          end
          p = 0.95; % probability mass enclosed insided the ellipsoid
          point_number = 100;  % number of point
          h = [];
          % holding = ishold;
          % holding = get(draw_to_these_axes,'NextPlot') ;
          if (C == zeros(2, 2))
              z = mu;
          else
              % Compute the Mahalanobis radius of the ellipsoid that encloses
              % the desired probability mass.
              k = obj.conf2mahal(p, 2);
              % The major and minor axes of the covariance ellipse are given by
              % the eigenvectors of the covariance matrix.  Their lengths (for
              % the ellipse with unit Mahalanobis radius) are given by the
              % square roots of the corresponding eigenvalues.
              %   if (issparse(Sigma))
              %     [V, D] = eigs(Sigma);
              %   else
              %     [V_s, D] = eig(Sigma);
              %   end
              %[~,D,V_s]=svd(C) ;
              [V_s, D] = eig(C);
              %% DEBUG
              %disp('principal directions module')
              %sqrt(abs(diag(D)))'
              %L1 = k * sqrt(abs(diag(D)));
              %L1'
              % Compute the points on the surface of the ellipse.
              t = linspace(0, 2*pi, point_number);
              u = [cos(t); sin(t)];
              w = (k * V_s * sqrt(D)) * u*obj.GetSigma();
              z = repmat(mu, [1 point_number]) + w;
               
              % Plot the major and minor axes.
              L = k * sqrt(abs(diag(D))) * obj.GetSigma();
              h = plot( [mu(1); mu(1) + L(1) * V_s(1, 1)], ...
                   [mu(2); mu(2) + L(1) * V_s(2, 1)], 'color', obj.clr);
              hold on;
              h = [h; plot( [mu(1); mu(1) + L(2) * V_s(1, 2)], ...
                       [mu(2); mu(2) + L(2) * V_s(2, 2)], 'color', obj.clr)];
          end

          h = [h; plot( z(1, :), z(2, :), 'color', obj.clr, 'LineWidth', 2)];
          % if (~holding) hold off; end
          % set(draw_to_these_axes,'NextPlot',holding);
      end
      
      % i plot all the old mean expect the last one
      function PlotTrace(obj)
          for i=1:(obj.current_index - 1)
              plot(obj.mean(i,1),obj.mean(i,2), 'rx', 'MarkerSize', 10,'color',obj.clr);
          end
      end
      
      function PlotBox(obj)
          [mu,V_s,tlb,tub] = obj.GetRotTraslBound();
          func = @(x_)obj.RotoTrasl(x_,mu,V_s);
          [X_trasl,Y_trasl] = meshgrid(linspace(tlb(1),tub(1),100),linspace(tlb(2),tub(2),100)); 
          xl_trasl = [X_trasl(:) Y_trasl(:)];
          [x_transf] = func(xl_trasl);
          plot(x_transf(:,1),x_transf(:,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
      end
      
   end
   
   methods(Static)
       %%%%%%%%%%%%

        % CONF2MAHAL - Translates a confidence interval to a Mahalanobis
        %              distance.  Consider a multivariate Gaussian
        %              distribution of the form
        %
        %   p(x) = 1/sqrt((2 * pi)^d * det(C)) * exp((-1/2) * MD(x, m, inv(C)))
        %
        %              where MD(x, m, P) is the Mahalanobis distance from x
        %              to m under P:
        %
        %                 MD(x, m, P) = (x - m) * P * (x - m)'
        %
        %              A particular Mahalanobis distance k identifies an
        %              ellipsoid centered at the mean of the distribution.
        %              The confidence interval associated with this ellipsoid
        %              is the probability mass enclosed by it.  Similarly,
        %              a particular confidence interval uniquely determines
        %              an ellipsoid with a fixed Mahalanobis distance.
        %
        %              If X is an d dimensional Gaussian-distributed vector,
        %              then the Mahalanobis distance of X is distributed
        %              according to the Chi-squared distribution with d
        %              degrees of freedom.  Thus, the Mahalanobis distance is
        %              determined by evaluating the inverse cumulative
        %              distribution function of the chi squared distribution
        %              up to the confidence value.
        %
        % Usage:
        % 
        %   m = conf2mahal(c, d);
        %
        % Inputs:
        %
        %   c    - the confidence interval
        %   d    - the number of dimensions of the Gaussian distribution
        %
        % Outputs:
        %
        %   m    - the Mahalanobis radius of the ellipsoid enclosing the
        %          fraction c of the distribution's probability mass
        %
        % See also: MAHAL2CONF

        % Copyright (C) 2002 Mark A. Paskin
       function m = conf2mahal(c, d)
            %% TODO 
            m = sqrt(chi2inv(c, d)); % matlab stats toolbox
            % pr = 0.341*2 ; c = (1 - pr)/2 ; norminv([c 1-c],0,1)
            
%             pr = c ; c = (1 - pr)/2 ; 
%             m = norminv([c 1-c],0,1) ;
%             m = m(2) ;
        end
       
   end
   
   
   
      
end