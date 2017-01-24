% the idea here is that this class is not gonna be owner of the istance (basically the fintess function and the constraints)
% but is gonna be a proxy to the gaussian process that model fitness and
% constraints and to the surrogate function.
%% this object just receive from outside the results of the simulations (fitness and constraints)
classdef BayesOpt < handle
    
    
    properties
        surrogate
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
        % for visualization 
        X_vis
        Y_vis
        xl_vis
        Z_vis
        z_constr
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

            % here i set up n_constraints + 1 GP fitness + 1 GP point
            % feasible or not (1 or 0)
            for i = 1:n_of_constraints + 2;
                if(strcmp(GP_lib,'GPML'))
                    self.gp_s{i} = GaussianProcess.GPML_GP();
                elseif(strcmp(GP_lib,'GP_stuff'))
                    self.gp_s{i} = GaussianProcess.GPstuff_GP();
                end
            end

            %% TODO define surrogate functions
            % Surrogate placeholder
            kind = 'ei';
            kappa =0.1;
            xi = 0;
            self.surrogate = @(self_,x_)Surrogate(self_, x_, kind, kappa, xi);

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
                name_real_function = 'to_test_withBOGP_stuff';
                fx = str2func(name_real_function);
                self.Z_vis = reshape(fx([],self.xl_vis),100,100);
                %% TODO pass the name of the function that we want to optimize from the outside 
                % precompute the constraints
                constr_function = {'stuffGPConstr1','stuffGPConstr1_1','stuffGPConstr2','stuffGPConstr2_1'};
                for ii = 1:n_of_constraints
                    cur_f = str2func(constr_function{ii});
                    val = cur_f(self.xl_vis,[]);
                    % I change the value for constraints (violation = nan, not violation = 1) 
                    val(val > 0) = nan;
                    val(~isnan(val))=1;
                    self.z_constr{ii} = reshape(val,100,100);
                end
            end
            % optimization ooptions for the surrogate function 
            %self.options_opt = optimoptions('fmincon','Algorithm','interior-point','Display','none','TolFun',1e-9,'TolX',1e-6);
            self.options_opt = optimoptions('fmincon','GradObj','off','Algorithm','trust-region-reflective','TolFun',1e-9,'TolX',1e-6);
            
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
            
            %% TODO i have to update y_max only if the the point is feasible
            index = (y_init(:,end-1)==1);
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
        
       
        function x_max = AcqMax(self)
    
        %     A function to find the maximum of the acquisition function using
        %     the 'L-BFGS-B' method.
        % 
        %     Parameters
        %     ----------
        %     Returns
        %     -------
        %     :return: x_max, The arg max of the acquisition function.


            % Start with the lower bound as the argmax
            x_max = self.bounds(1,:);  
            max_acq = -inf;

            %% TODO rand in bound (round is beetween 0 and 1)
            % i solve the optimization many time s to be sure that the optimal
            % solution is not local (i cannot remove this or i have to
            % change the optimzation method with a global approach)
            % if the surrogate has multiple optimal solutions this
            % procedure is necessary
            n_starting_point = 20;
            
            extended_lb = repmat(self.bounds(1,:),n_starting_point,1);
            extended_up = repmat(self.bounds(2,:),n_starting_point,1);
            % [a,b] -------> (b-a).*rand(n,1) + a;
            x_0 = (extended_up - extended_lb).*rand([n_starting_point,self.dim]) + extended_lb;
            
            
            %% TODO specify the structure of self.surrogate
            for i = 1:size(x_0,1)
                % Find the minimum of minus the acquisition function
               fun = @(x_)self.surrogate(self,x_);
               %% TODO Check if the surrogate function have the right sign (i want to maximize but im using a minimization)
               [x,fval] = fmincon(fun,x_0(i,:),[],[],[],[],self.bounds(1,:),self.bounds(2,:),[],self.options_opt);
               % Store it if better than previous minimum(maximum).
                if (-fval >= max_acq)
                    x_max = x;
                    max_acq = -fval;
                end

            end
            %% TODO extend clip to different bound per each dimension
            % Clip output to make sure it lies within the bounds. Due to floating
            % point technicalities this is not always the case.
            clip(x_max, self.bounds(1, :), self.bounds(2, :))
        end
        % here new y is a vector with the all the values [constraints violations,satisfy or not the constraints,fitness]
        function Update(self,new_x, new_y)
            
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
                    self.gp_s{i}.Update(new_x,new_y(i));
                end

                %% TODO add update of for y_max;
                if(new_y(end-1)==1)
                    if(new_y(end)>self.y_max) 
                        self.y_max = new_y(end);
                        self.x_max = new_x;
                    end
                end
            end

        end
        
        
         %GRAPHIC FUNCTION

 
%         %works only for function defined in R or R^(2) functions
%         %x and y are the value of the real function
% 
         function Plot(self,x_candidate)
             % compute current mean and variance for the objective
             % function;
             [ymu,ys2]=self.gp_s{end}.Predict(self.xl_vis);
             % compute the value for the surrogate function;
             fun = @(x_)self.surrogate(self,x_);
             sur = fun(self.xl_vis);
             clf
             
             
             % compute how many subploat
             img_col  = 4; % i set 4 col to have enough space for the fitness function visualization 
             img_rows = 1 + ceil((self.n_of_constraints*2)/img_col);  % i have 4 column i have to add rows depending on the number of the constraints (reconstructed plus original)
             
             
             % Plot the objective function
             subplot(img_rows,img_col,1),hold on, title('Objective, query points')
             box on
             pcolor(self.X_vis,self.Y_vis,self.Z_vis),shading flat
             clim = caxis;
             l1=plot(self.gp_s{end}.X(1:end,1),self.gp_s{end}.X(1:end,2), 'rx', 'MarkerSize', 10);
             %plot(x(end,1),x(end,2), 'ro', 'MarkerSize', 10)
             %l2=plot(xnews(:,1),xnews(:,2), 'ro', 'MarkerSize', 10);
             l3=plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
             legend([l1,l3], {'function evaluation points','The next query point'})
 
             % Plot the posterior mean of the GP model for the objective function
             subplot(img_rows,img_col,2),hold on, title(sprintf('GP prediction, mean,'))
             box on
             pcolor(self.X_vis,self.Y_vis,reshape(ymu,100,100)),shading flat
             caxis(clim)
             
             % Plot the posterior variance of GP model
             subplot(img_rows,img_col,3),hold on, title('GP prediction, variance')
             box on
             pcolor(self.X_vis,self.Y_vis,reshape(ys2,100,100)),shading flat
             %l2=plot(xnews(:,1),xnews(:,2), 'ro', 'MarkerSize', 10);
             l3=plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
             
             % Plot the expected improvement 
             subplot(img_rows,img_col,4), hold on, title(sprintf('Expected improvement %.2e', max(sur)))
             box on
             pcolor(self.X_vis,self.Y_vis,reshape(sur,100,100)),shading flat
             %plot(xnews(:,1),xnews(:,2), 'ro', 'MarkerSize', 10);
             plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
             
             % this is an index that i sue to control the position of the
             % subplot for the constraints
             subplot_position = 5;
             
             % i dislplay for each constraints its real value and its
             % reconstructed ones
             for counter = 1:self.n_of_constraints
                % real constraints 
                subplot(img_rows,img_col,subplot_position), hold on, title(sprintf('original constraints %.2e', counter))
                pcolor(self.X_vis,self.Y_vis,self.z_constr{counter}),shading flat    
                %plot(xc1(1:end-1,1),xc1(1:end-1,2), 'rx', 'MarkerSize', 10);
                plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10);
                %plot(xc1(end,1),xc1(end,2), 'ro', 'MarkerSize', 10, 'linewidth', 3);
                
                subplot_position = subplot_position + 1;
                % reconstructed constraints
                subplot(img_rows,img_col,subplot_position), hold on, title(sprintf('reconstructed constraints %.2e', counter))
                [cur_ymu,ys2]=self.gp_s{counter}.Predict(self.xl_vis);
                % i want to show the value wehre the constraints is
                % satisfacted
                cur_ymu(cur_ymu > 0) = nan;
                cur_ymu(~isnan(cur_ymu))=1;
                pcolor(self.X_vis,self.Y_vis,reshape(cur_ymu,100,100)),shading flat
                plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10)
                
                subplot_position = subplot_position + 1;
             end
             
             
%              counter = 1;
%              for j = 3:img_col
%                 ind = index(cur_row,j);
%                 subplot(img_rows,img_col,ind), hold on, title(sprintf('reconstructed constraints %.2e', counter))
%                 [cur_ymu,ys2]=self.gp_s{counter}.Predict(self.xl_vis);
%                 % i want to show the value wehre the constraints is
%                 % satisfacted
%                 cur_ymu(cur_ymu > 0) = nan;
%                 cur_ymu(~isnan(cur_ymu))=1;
%                 pcolor(self.X_vis,self.Y_vis,reshape(cur_ymu,100,100)),shading flat
%                 plot(x_candidate(1,1),x_candidate(1,2), 'ro', 'MarkerSize', 10)
%                 %plot(xc1(end,1),xc1(end,2), 'ro', 'MarkerSize', 10, 'linewidth', 3)
%                 %plot(xc1(1:end-1,1),xc1(1:end-1,2), 'rx'), 
%                 counter = counter + 1;
%              end    
             
             
             
         end

    end
end
              
                