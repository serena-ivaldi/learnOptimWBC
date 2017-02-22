% the idea here is that this class is not gonna be owner of the istance (basically the fintess function and the constraints)
% but is gonna be a proxy to the gaussian process that model fitness and
% constraints and to the surrogate function.
%% TODO fix the not psotive define gramiam matrix issue
%% this object just receive from outside the results of the simulations (fitness and constraints)
%% this class allows for the "lazy update" of the Internal matrixes and the hyperparameters of the GPs
classdef BayesOpt < handle
    
    
    properties
        surrogate
        xi                % parameters used inside the surrogate function to balance exploration and exploitation
        kappa             % parameters used inside the surrogate function to balance exploration and exploitation
        bounds
        dim
        n_of_constraints
        gp_s              % [GP_constraints, GP_satisfaction(1 or 0), GP_fitness]
        initialized
        eps 
        verbose
        y_max 
        x_max
        pd    %  i need it for the surrogate computation.
        radius
        options_opt
        %current_constraints_to_optimize   % maybe im gonna remove this feature
        % for visualization 
        X_vis
        Y_vis
        xl_vis
        Z_vis
        z_constr
        kind
        min_or_max
        temporary_lb
        temporary_ub
        local_sampling_distribution % with this variable i specify if the distribution to sample the initial point to optimize when i do the local GP 
        d_par      % distribution parameter for local sampling of the starting point in the optimization: in this struct i store all the parameter that i need to compute the disitribution at the current iteration
    end
    
    methods
        function self = BayesOpt(lb, ub, dim, n_of_constraints,GP_lib,varargin)

            %   param f: Function to be maximized.
            %   param verbose: Whether or not to print progress.


            % Find number of parameters
            self.dim = dim;
            self.n_of_constraints = n_of_constraints;

            % Create an array with parameters bounds
            self.bounds(1,:) = lb;
            self.bounds(2,:) = ub;

            % Initialization flag
            self.initialized = false;

            % here i set up n_constraints + 1 GP fitness + 1 GP about
            % constraints satisfaction
            for i = 1:n_of_constraints + 2;
                if(strcmp(GP_lib,'GPML'))
                    self.gp_s{i} = GaussianProcess.GPML_GP();
                elseif(strcmp(GP_lib,'GP_stuff'))
                    self.gp_s{i} = GaussianProcess.GPstuff_GP();
                end
            end

            %% TODO define surrogate functions
            % Surrogate placeholder
            self.kind = 'ecv';
            self.SetMinMax();
            self.kappa =0.01;
            self.xi = 0;
            self.surrogate = @(self_,x_)Surrogate(self_, x_);
   
            % PrintLog object
            %self.plog = PrintLog(self.keys)
            %tolerance for equality constraints 
            %% TODO add eps and verbose in a meaningfull way
            self.eps = 0.1;
            % Verbose
            self.verbose = true;
            
            % we need to initialize y_max in the case that during the
            % initialization no y_max is found ( i put a generic small value it should be adaptive though)
            self.y_max = -100000;
            self.x_max = [];
            
            self.pd =  makedist('Normal');
            %% TODO add radius as parameter of the method
            self.radius = 0;
            
            % for visualization
            if(dim <= 2)
                [self.X_vis,self.Y_vis] = meshgrid(linspace(lb(1),ub(1),100),linspace(lb(2),ub(2),100));
                self.xl_vis = [self.X_vis(:) self.Y_vis(:)];
                %% TODO pass the name of the function that we want to optimize from the outside 
                name_real_function = 'g06'; % to_test_withBOGP_stuff g06
                fx = str2func(name_real_function);
                self.Z_vis = reshape(fx([],self.xl_vis),100,100);
                %% TODO pass the name of the function that we want to optimize from the outside 
                % precompute the constraints
                constr_function = {'g06Constr1','g06Constr2'};  %{'stuffGPConstr1_0','stuffGPConstr1_1','stuffGPConstr2_1'}; {'g06Constr1','g06Constr2'}
                %const = [0 0.8 ; -10 0.1];
                for ii = 1:n_of_constraints
                    cur_f = str2func(constr_function{ii});
                    val = cur_f(self.xl_vis,[]);
                    % I change the value for constraints (violation = nan, not violation = 1) 
                    val(val > 0) = nan;
                    %val(val<const(ii,1) | val>const(ii,2)) = nan;
                    val(~isnan(val))=1;
                    self.z_constr{ii} = reshape(val,100,100);
                end
            end
            % optimization ooptions for the surrogate function 
            %self.options_opt = optimoptions('fmincon','Algorithm','interior-point','Display','none','TolFun',1e-9,'TolX',1e-6);
            self.options_opt = optimoptions('fmincon','GradObj','off','Algorithm','interior-point','TolFun',1e-9,'TolX',1e-6,'Display','none');
            % defautl for now is gaussian
            self.local_sampling_distribution = 'gaussian';
            self.d_par = [];
        end
        
        function Init(self, x_init,y_init)

        %  Initialization method to kick start the optimization process. It is a
        %  combination of points passed by the user, and randomly sampled ones.
        % 
        %  param init_points: Number of random points to probe.
            %% TODO check it
            for i = 1:length(self.gp_s)
                self.gp_s{i}.Init(x_init,y_init(:,i));
            end   
            
            %% TODEBUG im cheching if i have to use the best y regardless or i 
            %% i have to check for the best y max that satisfy the constraints
            %index = (y_init(:,end-1)==1);
            index = ones(size(y_init));
            [y_candidate, ind_max] = max(y_init(index,i));     
            if(~isempty(y_candidate))
                self.y_max = y_candidate;
                self.x_max = x_init(ind_max,:);
            end
        end

    %    function initialize(self, points_dict):
    %        """
    %        Method to introduce point for which the target function
    %        value is known
    %
    %        :param points_dict:
    %        :return:
    %        """
    %
    %        for target in points_dict:
    %
    %            self.y_init.append(target)
    %
    %            all_points = []
    %            for key in self.keys:
    %                all_points.append(points_dict[target][key])
    %
    %            self.x_init.append(all_points)

        %function SetBounds(self, new_bounds)

         % A method that allows changing the lower and upper searching bounds
         % 
         % param new_bounds: A dictionary with the parameter name and its new
        %end
        
        %% varargin(1) 
        %% varargin(2)
        %% varargin(3)
        function [x_max, max_acq] = AcqMax(self,varargin)
    
        %     A function to find the maximum of the acquisition function using
        %     the 'L-BFGS-B' method.
        % 
        %     Parameters
        %     ----------
        %     Returns
        %     -------
        %     :return: x_max, The arg max of the acquisition function.
            
            % this modified bound are related to the traslation
            if length(varargin) == 3
                lb = varargin{1};
                ub = varargin{2};
                %% for visualization only
                self.temporary_lb = varargin{1};
                self.temporary_ub = varargin{2};
            else
                lb = self.bounds(1,:);
                ub = self.bounds(2,:);
            end

            % Start with the lower bound as the argmax
            x_max = lb;  
            if(strcmp(self.min_or_max,'max'))
                max_acq = -inf;
            else
                max_acq = inf;
            end

            %% TODO rand in bound (rand is beetween 0 and 1)
            %% TODO set the n_starting_point as a parameter of the algorithm
            % i solve the optimization many time s to be sure that the optimal
            % solution is not local (i cannot remove this or i have to
            % change the optimzation method with a global approach)
            % if the surrogate has multiple optimal solutions this
            % procedure is necessary
            n_starting_point = 40;
            
            
            % [a,b] -------> (b-a).*rand(n,1) + a;
            %% to do i need to add a check to verify if the bounding box of the local search is all insde the bounding box of the global functions
            %% in that case i can skip this selection procedure
            if (length(varargin) == 3)
                %% for local search
                % i need to do this to select the point that are 
                new_n_starting_point = n_starting_point*5;
                x_0=[];
                %% equiprobable distribution
                extended_lb = repmat(lb,new_n_starting_point,1);
                extended_ub = repmat(ub,new_n_starting_point,1);
                minaction = repmat(self.bounds(1,:),new_n_starting_point,1);
                %maxaction = repmat(self.bounds(2,:),new_n_starting_point,1);
                % i keep adding point till i reach "n_starting_point" points
                while(length(x_0)<n_starting_point)
                    if(strcmp(self.local_sampling_distribution,'equiprobable'))
                        x_test = self.EquiprobableDistribution(new_n_starting_point,extended_lb,extended_ub);
                    elseif(strcmp(self.local_sampling_distribution,'gaussian'))
                        x_test = self.GaussianDistribution(new_n_starting_point);
                    end
                    x_test_transf = varargin{3}(x_test);
                    %% in this index i have all the point that satisfy minaction
                    index_min = prod( (x_test_transf > minaction),2 );
                    index_min = find(index_min == 1);
                    x_test_transf = x_test_transf(index_min,:);
                    x_test = x_test(index_min,:);
                    maxaction = repmat(self.bounds(2,:),size(x_test,1),1);
                    index_max = prod( (x_test_transf < maxaction),2 );
                    index_max = find(index_max == 1);
                    x_test = x_test(index_max,:);
                    % add te good point to the starting point
                    x_0 = [x_0;x_test];
                end
                % remove the point in excess
%                 cur_len = length(x_0); 
%                 if(cur_len>n_starting_point)
%                     excess = cur_len - n_starting_point;
%                     index = randi(cur_len,excess,1);
%                     x_0(index,:) = [];
%                 end
            else
                %% for global search (here i only use equirobable distribution to sample starting point)
                extended_lb = repmat(lb,n_starting_point,1);
                extended_ub = repmat(ub,n_starting_point,1); 
                x_0 = (extended_ub - extended_lb).*rand([n_starting_point,self.dim]) + extended_lb;
            end
            
            %% TODO specify the structure of self.surrogate
            for i = 1:n_starting_point
                % Find the minimum of minus the acquisition function
               fun = @(x_)self.surrogate(self,x_);
               %% TODO Check if the surrogate function have the right sign (i want to maximize but im using a minimization)
               [x,fval] = fmincon(fun,x_0(i,:),[],[],[],[],lb,ub,[],self.options_opt);
               % Store it if better than previous minimum(maximum).
               if(strcmp(self.min_or_max,'max'))
                   if (-fval >= max_acq)
                        x_max = x;
                        max_acq = -fval;
                   end
               else
                   if (fval <= max_acq)
                        x_max = x;
                        max_acq = fval;
                   end
               end

            end
            %% TODO extend clip to different bound per each dimension
            % Clip output to make sure it lies within the bounds. Due to floating
            % point technicalities this is not always the case.
            clip(x_max, lb, ub);
        end
        
        function TrainGPs(self)
            for i=1:length(self.gp_s)
                 self.gp_s{i}.Train();
            end
        end
        % here new y is a vector with the all the values [constraints violations,satisfy or not the constraints,fitness]
        function Update(self,new_x, new_y,train)
            % Find unique rows of X to avoid GP from breaking
            X = self.gp_s{1}.X;
            mat_x_new = repmat(new_x,size(X,1),1);
            mat_diff = X - mat_x_new;
            mat_difftwo = mat_diff .* mat_diff;
            mat_dist = sum(mat_difftwo,2);
            mat_dist = sqrt(mat_dist);
            index = mat_dist>self.radius;
            uniqueness = prod(index);
            %% TODO i have to change this part in a way at least the value pf ymax and xmax are updated 
            %% on the other side i need to assure that new point are added (adding a little bit of noise for example)
            if(uniqueness)
                for i=1:length(self.gp_s)     
                    self.gp_s{i}.Update(new_x,new_y(i),train);
                end

                %% TODEBUG checking if updating the ymax without checking feasibility is better than update the same 
                %% taking into account only the ymax feasible
                %if(new_y(end-1)==1)
                    if(new_y(end)>self.y_max) 
                        self.y_max = new_y(end);
                        self.x_max = new_x;
                    end
                %end
            end

        end     
        
        function SetMinMax(self,varargin)
            
            if strcmp(self.kind,'custom');
                handle_name = func2str(varargin{1});
                [a,b]=strtok(handle_name,'.');
                [function_name]=strtok(b,'(');
                function_name(1) = []; 
            else
                function_name = self.kind;
            end
            
            % function list name 
            if strcmp(function_name,'ucb')
               self.min_or_max = 'max';
            
            elseif strcmp(function_name,'ei')
               self.min_or_max = 'max';
            
            elseif strcmp(function_name,'poi')
               self.min_or_max = 'max';
            
            elseif strcmp(function_name,'eci')
               self.min_or_max = 'max';
            
            elseif strcmp(function_name,'ecv')
               self.min_or_max = 'max';
            
    %         if strcmp(function_name,'ec')
    %             [ret, x] =  self.ec(x);
    %             ret = - ret;
    %         end
    %         if(strcmp(function_name,'ecm'))
    %             % no need to invert the sign i have to minize this funcition
    %             [ret, x] =  self.ecm(x);
    %         end
            elseif(strcmp(function_name,'pcs_constr'))
               self.min_or_max = 'max';
            
            elseif(strcmp(function_name,'ucb_constr'))
               self.min_or_max = 'min';
            
            elseif strcmp(function_name,'mcd_constr')
                self.min_or_max = 'max';
                
            elseif strcmp(function_name,'cucb')
               self.min_or_max = 'max';
            end
        end
        % use this function to change the surrogate function during the
        % execution from constructor default is 'ecv'
        % kind is a string
        %% for kind = custom the first vararging is reservef to the function handle for the zooming
        function SetSurrogate(self,kind,varargin)
            %% TODO change this value from outside
            self.kappa =0.1;
            self.xi = 0;
            self.kind = kind;
            self.SetMinMax(varargin{:});
            self.surrogate = @(self_,x_)Surrogate(self_, x_,varargin{:});
        end
        
        
        function ZoomingIn(self,mu,tub)
             % for the radius i get 
            for i=1:length(self.gp_s)
                 self.gp_s{i}.ActivateZooming(mu,max(tub))
            end
        end
        
        function ZoomingOut(self)
            for i=1:length(self.gp_s)
                 self.gp_s{i}.DeactivateZooming();
            end
        end
        
        function SetSampleDistribution(self,distr)
            self.local_sampling_distribution = distr;
        end
        
        %% TODO this section has to be organized in a proper way now is quite crappy!
        function SetSampleDistributionParam(self,varargin)
            if(strcmp(self.local_sampling_distribution,'equiprobable'))
                %self.d_par.lb = varargin{1};
                %self.d_par.ub = varargin{2};
            elseif(strcmp(self.local_sampling_distribution,'gaussian'))
                self.d_par.A = varargin{1};      % this is the cholesky factor of the covariance matrix 
                self.d_par.mean = varargin{2};
                self.d_par.sigma = varargin{3};
                self.d_par.sigma_multiplier = varargin{4};
                self.d_par.lb = varargin{5};
                self.d_par.ub = varargin{6};
            end
        end
        
        %% distribution to sample starting point for local GP optimization
        %% TODO i should put all the disitribution togheter in one function
        function x = EquiprobableDistribution(self,n_points,lb,ub)
            x = (ub - lb).*rand([n_points,self.dim]) + lb;
        end
        
        function x = GaussianDistribution(self,n_points)
            x = zeros(n_points,self.dim);
            % i set the mean at zero if the value is empty
            if(isempty(self.d_par.mean))
                self.d_par.mean = zeros(1,self.dim);
            end
            for i=1:n_points
               z =  mvnrnd(zeros(1, self.dim), eye(self.dim));
               candidate = self.d_par.mean + self.d_par.sigma * self.d_par.sigma_multiplier *(self.d_par.A * z')';
               %% DEBUG
               if(~isreal(candidate))
                   disp('errore x non reale');
               end
               % saturation
               candidate(1, candidate(1,:) > self.d_par.ub) = self.d_par.ub(candidate(1,:) > self.d_par.ub);
               candidate(1, candidate(1,:) < self.d_par.lb) = self.d_par.lb(candidate(1,:) < self.d_par.lb);
               x(i,:) = candidate;
            end
        end
         %% GRAPHIC FUNCTION

 
%         %works only for function defined in R or R^(2) functions
%         %x and y are the value of the real function
% 
         function Plot(self,x_candidate)
             % compute current mean and variance for the objective
             % function;
             [ymu,ys2]=self.gp_s{end}.Predict(self.xl_vis);
              % compute the value for the surrogate function;
             fun = @(x_)self.surrogate(self,x_);
             if(strcmp(self.kind,'custom'))
                not_obsolete_surrogate = true; 
                [X_trasl,Y_trasl] = meshgrid(linspace(self.temporary_lb(1),self.temporary_ub(1),100),linspace(self.temporary_lb(2),self.temporary_ub(2),100)); 
                xl_trasl = [X_trasl(:) Y_trasl(:)];
                try
                    [sur, x_transf] = fun(xl_trasl);
                catch
                    disp('obsolete surrogate function due to cancellation of particles, i cannot print this function')
                    not_obsolete_surrogate = false;
                end
             else    
                [sur, x_transf] = fun(self.xl_vis);
             end
             %clf
                          
             % compute how many subploat
             img_col  = 4; % i set 4 col to have enough space for the fitness function visualization 
             img_rows = 1 + ceil((self.n_of_constraints*2)/img_col);  % i have 4 column i have to add rows depending on the number of the constraints (reconstructed plus original so i multiply by 2)
             
             figure
             % Plot the objective function
             subplot(img_rows,img_col,1),hold on, title('Objective, query points')
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             pcolor(self.X_vis,self.Y_vis,self.Z_vis),shading flat
             clim = caxis;
             l1=plot(self.gp_s{end}.X(1:end,1),self.gp_s{end}.X(1:end,2), 'rx', 'MarkerSize', 10);
             %plot(x(end,1),x(end,2), 'ro', 'MarkerSize', 10)
             %l2=plot(xnews(:,1),xnews(:,2), 'ro', 'MarkerSize', 10);
             l3=plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
             %legend([l1,l3], {'function evaluation points','The next query point'})
 
             % Plot the posterior mean of the GP model for the objective function
             subplot(img_rows,img_col,3),hold on, title(sprintf('GP prediction, mean,'))
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             pcolor(self.X_vis,self.Y_vis,reshape(ymu,100,100)),shading flat
             %caxis(clim)
             
             % Plot the posterior variance of GP model
             subplot(img_rows,img_col,4),hold on, title('GP prediction, variance')
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             pcolor(self.X_vis,self.Y_vis,reshape(ys2,100,100)),shading flat
             %l2=plot(xnews(:,1),xnews(:,2), 'ro', 'MarkerSize', 10);
             l3=plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
             
             % Plot current surrogate function;  
             subplot(img_rows,img_col,2), hold on, title(sprintf(strcat('Surrogate function kind = ', self.kind)))
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             if(strcmp(self.kind,'custom') && not_obsolete_surrogate)
                 %blank = zeros(size(self.X_vis));
                 %pcolor (self.X_vis,self.Y_vis,blank),shading flat
                 pcolor(reshape(x_transf(:,1),100,100),reshape(x_transf(:,2),100,100),reshape(sur,100,100)),shading flat
             else
%                 pcolor(self.X_vis,self.Y_vis,reshape(sur,100,100)),shading flat
             end
             
             %plot(xnews(:,1),xnews(:,2), 'ro', 'MarkerSize', 10);
             plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
             
             % this is an index that i sue to control the position of the
             % subplot for the constraints
             subplot_position = 5;
             %const = [0 0.8 ; -10 0.1];
             % i dislplay for each constraints its real value and its
             % reconstructed ones
             for counter = 1:self.n_of_constraints
                % real constraints 
                subplot(img_rows,img_col,subplot_position), hold on, title(sprintf('original constraints %.2e', counter))
                axis normal ;
                axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
                pcolor(self.X_vis,self.Y_vis,self.z_constr{counter}),shading flat    
                %plot(xc1(1:end-1,1),xc1(1:end-1,2), 'rx', 'MarkerSize', 10);
                plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10);
                %plot(xc1(end,1),xc1(end,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                
                subplot_position = subplot_position + 1;
                % reconstructed constraints
                subplot(img_rows,img_col,subplot_position), hold on, title(sprintf('reconstructed constraints %.2e', counter))
                axis normal ;
                axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
                [cur_ymu,ys2]=self.gp_s{counter}.Predict(self.xl_vis);
                % i want to show the value wehre the constraints is
                % satisfacted
                cur_ymu(cur_ymu > 0) = nan;
                %cur_ymu(cur_ymu<const(counter,1) | cur_ymu>const(counter,2)) = nan;
                cur_ymu(~isnan(cur_ymu))=1;
                pcolor(self.X_vis,self.Y_vis,reshape(cur_ymu,100,100)),shading flat
                plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10)
                
                subplot_position = subplot_position + 1;
             end 
             
         end
         
         function PlotArtificial(self)
              %% print of artificial constraints
             figure
             gp_test = self.gp_s{end - 1};
             [ymu_a,ys2_a]=self.gp_s{end - 1}.Predict(self.xl_vis);
             subplot(1,3,2),hold on, title('artificial constraints mean')
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             pcolor(self.X_vis,self.Y_vis,reshape(ymu_a,100,100)),shading flat
             %caxis(clim)
             
             subplot(1,3,3),hold on, title('artificial constraints variance')
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             pcolor(self.X_vis,self.Y_vis,reshape(ys2_a,100,100)),shading flat
             %caxis(clim)
             
             ymu_a(ymu_a > 0) = nan;
             ymu_a(~isnan(ymu_a))=1;
             
             subplot(1,3,1),hold on, title('artificial constraints free regions')
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             pcolor(self.X_vis,self.Y_vis,reshape(ymu_a,100,100)),shading flat
             %caxis(clim)
         end
         
         function PlotSurrogateByKind(self,kind,varargin)
              %% print of artificial constraints
             figure
             cur_surrogate = self.surrogate;
             SetSurrogate(self,kind,varargin(:));
             fun = @(x_)self.surrogate(self,x_);
             if(strcmp(kind,'custom'))
                not_obsolete_surrogate = true; 
                [X_trasl,Y_trasl] = meshgrid(linspace(self.temporary_lb(1),self.temporary_ub(1),100),linspace(self.temporary_lb(2),self.temporary_ub(2),100)); 
                xl_trasl = [X_trasl(:) Y_trasl(:)];
                try
                    [sur, x_transf] = fun(xl_trasl);
                catch
                    disp('obsolete surrogate function due to cancellation of particles, i cannot print this function');
                    not_obsolete_surrogate = false;
                end
             else    
                [sur, x_transf] = fun(self.xl_vis);
             end
             box on
             axis normal ;
             axis([self.bounds(1,1),self.bounds(2,1),self.bounds(1,2),self.bounds(2,2)])
             if(strcmp(kind,'custom') && not_obsolete_surrogate)
                 pcolor(reshape(x_transf(:,1),100,100),reshape(x_transf(:,2),100,100),reshape(sur,100,100)),shading flat
             else
                 disp('i cannot print the particle')
                 %pcolor(self.X_vis,self.Y_vis,reshape(sur,100,100)),shading flat
             end
             % set back the old surrogate
             self.surrogate = cur_surrogate;
         end
         
    

    end
end
              
                