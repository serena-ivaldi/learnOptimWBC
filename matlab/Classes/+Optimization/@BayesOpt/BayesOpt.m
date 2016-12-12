% the idea here is that this class is not gonna be owner of the istance (basically the fintess function and the constraints)
% but is gonna be a proxy to the gaussian process that model fitness and
% constraints and to the surrogate function.
%% this object just receive from outside the results of the simulations (fitness and constraints)
classdef BayesOpt
    
    
    properties
        surrogate
        pbounds
        dim
        gp_s           % the hyp here is that the last one is the one with the fitness function
        init_points
        eps 
        verbose
        y_max 
    end
    
    methods
        function self = BayesianOptimization(instance, lb, ub, dim, n_of_constraints, varargin)

            %   param f: Function to be maximized.
            %   param verbose: Whether or not to print progress.


            % Find number of parameters
            self.dim = dim;

            % Create an array with parameters bounds
            self.bounds(1,:) = lb;
            self.bounds(2,:) = ub;

            % Initialization flag
            self.initialized = false  

            % here i set up n_constraints + 1 gaussian process 
            for i = 1:n_of_constraints + 1;
                self.gp_s{i} = GaussianProcess.GPML_GP()
            end

            %% TODO define surrogate functions
            % Surrogate placeholder
            self.surrogate = @(self,x)Surrogate(self, x, kind, kappa, xi);

            % PrintLog object
            %self.plog = PrintLog(self.keys)
            %tolerance for equality constraints 
            self.eps = eps;
            % Verbose
            self.verbose = verbose
        end
        
        function Init(self, x_init,y_init)

        %  Initialization method to kick start the optimization process. It is a
        %  combination of points passed by the user, and randomly sampled ones.
        % 
        %  param init_points: Number of random points to probe.
        %% TODO check it
            for i = 1:dim(self.gp_s,1)
                gp_s{i}.init(x_init,y_init(:,i));
                if(i==dim(self.gp_s,1))
                    %% TODO i have to update y_max only if the the point is feasible
                    self.y_max = max(y_init(:,i));
                end
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

        function SetBounds(self, new_bounds)

         % A method that allows changing the lower and upper searching bounds
         % 
         % param new_bounds: A dictionary with the parameter name and its new
        end
        
       
        function x_max = AcqMax(self)
    
        %     A function to find the maximum of the acquisition function using
        %     the 'L-BFGS-B' method.
        % 
        %     Parameters
        %     ----------
        %     :param ac:
        %         The acquisition function object that return its point-wise value.
        % 
        %     :param gp:
        %         A gaussian process fitted to the relevant data.
        % 
        %     :param y_max:
        %         The current maximum known value of the target function.
        % 
        %     :param bounds:
        %         The variables bounds to limit the search of the acq max.
        % 
        % 
        %     Returns
        %     -------
        %     :return: x_max, The arg max of the acquisition function.


            % Start with the lower bound as the argmax
            x_max = self.bounds(1,:);  
            max_acq = -inf;

            %%% TODO rand in bound (round is beetween 0 and 1)
            % i solve the optimization many time s to be sure that the optimal
            % solution is not local (i could remove this )
            x_0 = rand([100,self.dim]);

            %%% TODO specify the structure of self.surrogate
            for i = 1: length(x_0)
                % Find the minimum of minus the acquisition function
               fun = self.surrogate;
               [x,fval] = fmincon(fun,x0(i,:),[],[],[],[],self.bounds(1,:),self.bounds(2,:));


                % Store it if better than previous minimum(maximum).
                if (-res.fun >= max_acq)
                    x_max = x;
                    max_acq = fval;
                end

            end
            %%% TODO extend clip to different bound per each dimension
            % Clip output to make sure it lies within the bounds. Due to floating
            % point technicalities this is not always the case.
            clip(x_max, bounds(1, :), bounds(2, :))
        end
        % here new y is a vector with the all the values [constraints violations, fitness/ satisfy or not the constraints]
        function Update(self,new_x, new_y)
            
            
            for i=1:  length(self.gp_s)
                self.gp_s{i}.update(new_x,new_y(i));
            end
            %% TODO check for unique rows in the gp
            % Find unique rows of X to avoid GP from breaking
            %% TODO add update of for y_max;

        end
        
        
        function maximize(self,
                     init_points=5,
                     n_iter=25,
                     acq='ei',
                     kappa=2.576,
                     xi=0.0,
                     risk = 0.0,
                     **gp_params)

            %Main optimization method.

    %         Parameters
    %         ----------
    %         :param init_points:
    %             Number of randomly chosen points to sample the
    %             target function before fitting the gp.
    % 
    %         :param n_iter:
    %             Total number of times the process is to repeated. Note that
    %             currently this methods does not have stopping criteria (due to a
    %             number of reasons), therefore the total number of points to be
    %             sampled must be specified.
    % 
    %         :param acq:
    %             Acquisition function to be used, defaults to Expected Improvement.
    % 
    %         :param gp_params:
    %             Parameters to be passed to the Scikit-learn Gaussian Process object
    % 
    %         Returns
    %         -------
    %         :return: Nothing

            % Reset timer
            self.plog.reset_timer()

            % Set acquisition function
            self.util = 

            % Print new header
            if self.verbose
                self.plog.print_header(initialization=False)
            end
            % Iterative process of searching for the maximum. At each round the
            % most recent x and y values probed are added to the X and Y arrays
            % used to train the Gaussian Process. Next the maximum known value
            % of the target function is found and passed to the acq_max function.
            % The arg_max of the acquisition function is found and this will be
            % the next probed value of the target function in the next round.
            for i in range(n_iter)
                % Test if x_max is repeated, if it is, draw another one at random
                % If it is repeated, print a warning
                pwarning = False
                if np.any((self.X - x_max).sum(axis=1) == 0):

                    % estraggo un punto? si
                    x_max = np.random.uniform(self.bounds[:, 0],
                                              self.bounds[:, 1],
                                              size=self.bounds.shape[0])

                    pwarning = True
                end
                self.Update(x_max)

                % Update maximum value to search for next probe point.
                % check the last point
                % TODO rewrite the way which the y_max is updated taking into account the 
                % case that we are not in the admissibile region
                %if self.Y[-1] > y_max:
                %    y_max = self.Y[-1]



                % TODO put the x_plot and out_plot in the constructor
                % TODO use another flag for the visualization 
                if self.verbose
                    variate_one = np.linspace(0, 4, 100)
                    variate_two = np.linspace(0, 4, 100)
                    xv, yv = np.meshgrid(variate_one, variate_two) 
                    %list of matrix
                    x_plot = [xv, yv]
                    out_plot = self.f(xv,yv)
                    self.plot_gp(x_plot , out_plot , True, i)
                end
  
                
            end

        end
        %GRAPHIC FUNCTION

        function [mu,sig] = posterior(self,x,multivariate)
            %xmin, xmax = -2, 10
            %self.gp.fit(self.X, self.Y)
            if (~multivariate) 
                x_ = np.atleast_2d(x).T
                mu, sigma2 = self.gp.predict(x_, eval_MSE=True)
            else
                % unroll the meshgrid matrix
                xv_c = np.atleast_2d(x[0].ravel('F')).T
                yv_c = np.atleast_2d(x[1].ravel('F')).T 
                x_ = np.hstack((xv_c,yv_c))
                mu, sigma2 = self.gp.predict(x_, eval_MSE=True)
                dims = x[0].shape
                mu = np.reshape(mu, dims).T % C-like index orderingd
            end
            sig = np.sqrt(sigma2)
        end

        %works only for function defined in R or R^(2) functions
        %x and y are the value of the real function

        function plot_gp(self , x , out , multivariate, iterationID)

            if not multivariate
                fig = plt.figure(figsize=(16, 10))
                fig.suptitle('Gaussian Process and Utility Function After {} Steps'.format(len(self.X)), fontdict={'size':30})

                gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1]) 
                axis = plt.subplot(gs[0])
                acq = plt.subplot(gs[1])

                mu, sigma = self.posterior(x)
                axis.plot(x, out, linewidth=3, label='Target')
                axis.plot(self.X.flatten(), self.Y, 'D', markersize=8, label=u'Observations', color='r')
               % axis.plot(x, mu, '--', color='k', label='Prediction')

                axis.fill(np.concatenate([x, x[::-1]]), 
                          np.concatenate([mu - 1.9600 * sigma, (mu + 1.9600 * sigma)[::-1]]),
                    alpha=.6, fc='c', ec='None', label='95% confidence interval')

                axis.set_xlim((x[0], x[x.shape[0] - 1]))
                axis.set_ylim((None, None))
                axis.set_ylabel('f(x)', fontdict={'size':20})
                axis.set_xlabel('x', fontdict={'size':20})

                utility = self.util.utility(x.reshape((-1, 1)), self.gp, 0)
                acq.plot(x, utility, label='Utility Function', color='purple')
                acq.plot(x[np.argmax(utility)], np.max(utility), '*', markersize=15, 
                         label=u'Next Best Guess', markerfacecolor='gold', markeredgecolor='k', markeredgewidth=1)
                acq.set_xlim((x[0], x[x.shape[0] - 1]))
                acq.set_ylim((0, np.max(utility) + 0.5))
                acq.set_ylabel('Utility', fontdict={'size':20})
                acq.set_xlabel('x', fontdict={'size':20})

                axis.legend(loc=2, bbox_to_anchor=(1.01, 1), borderaxespad=0.)
                acq.legend(loc=2, bbox_to_anchor=(1.01, 1), borderaxespad=0.)   
            else
                fig = plt.figure(figsize=(10, 10))
                fig.suptitle('Gaussian Process and Utility Function After {} Steps'.format(len(self.X)), fontdict={'size':30})
                gs = gridspec.GridSpec(4, 1) 
                real = plt.subplot(gs[0])
                surr_plot = plt.subplot(gs[1])
                acq = plt.subplot(gs[2]) 
                var_plot = plt.subplot(gs[3])

                % real heat map
                heatmap = real.pcolor(x[0],x[1],out, cmap='seismic')
                plt.colorbar(heatmap)
                %compute posterior            
                mu, sigma = self.posterior(x,multivariate)

                index_starting_point = len(self.init_points) - 1;
                X_new = self.X[(index_starting_point + 1):];
                Y_ = self.clf.predict(X_new);
                color_iter = it.cycle(['r', 'g', 'b', 'c', 'm'])

                %%% Plot the surrogate function

                % plot differently the point belonging to the starting set 
                % and the one acquired during the exploration
                surr_plot.plot(self.X[0:index_starting_point,0].flatten(),self.X[0:index_starting_point,1].flatten(),'D', markersize=8, label=u'Observations', color='r')
                surr_plot.plot(self.X[(index_starting_point + 1):,0].flatten(),self.X[(index_starting_point + 1):,1].flatten(),'D', markersize=8, label=u'Observations', color='g')
                heatmap = surr_plot.pcolor(x[0],x[1],mu,cmap='seismic') 
                plt.colorbar(heatmap)
                for i, (mean, covar, color) in enumerate(zip(self.clf.means_, self.clf.covariances_, color_iter))
                        v, w = linalg.eigh(covar)
                        u = w[0] / linalg.norm(w[0])
                        % as the DP will not use every component it has access to
                        % unless it needs it, we shouldn't plot the redundant
                        % components.
                        if  (~np.any(Y_ == i))
                            continue
                        end
                        surr_plot.autoscale(enable=False)
                        surr_plot.scatter(X_new[Y_ == i, 0], X_new[Y_ == i, 1], 8, color=color, marker='o')
                        % Plot an ellipse to show the Gaussian component
                        angle = np.arctan(u[1] / u[0])
                        angle = 180 * angle / np.pi  % convert to degrees
                        ell = mtlpltpatches.Ellipse(mean, v[0], v[1], 180 + angle, color=color)
                        ell.set_clip_box(surr_plot.bbox)
                        ell.set_alpha(0.5)
                        surr_plot.add_artist(ell)      

                end

                %axis.set_xlim((x[0], x[x.shape[0] - 1]))
                %axis.set_ylim((None, None))

                % Plot the constraints
    %            xRow = np.atleast_2d(x[0].ravel('F')).T
    %            yRow = np.atleast_2d(x[1].ravel('F')).T
    %            xy   = np.hstack((xRow, yRow))
    %            dims = x[0].shape            
    %            admits  = []
    %            for row in xy:
    %                admits.append(admissibility(row, self.constraints, self.constraints_type, self.keys,self.eps))
    %            for xIter, yIter, admitsIter in zip( np.nditer(xRow),  np.nditer(yRow), admits):
    %                if admitsIter == 1:
    %                    acq.plot(xIter, yIter, '*', markersize=10, 
    %                     label=u'Next Best Guess', markerfacecolor='green', markeredgecolor='k', 
    %                     markeredgewidth=1, alpha=0.05)
    %                    surr_plot.plot(xIter, yIter, '*', markersize=10, 
    %                     label=u'Next Best Guess', markerfacecolor='green', markeredgecolor='k', 
    %                     markeredgewidth=1, alpha=0.05)

                %%% Plot the acquisition function
                xv_c = np.atleast_2d(x[0].ravel('F')).T
                yv_c = np.atleast_2d(x[1].ravel('F')).T 
                x_ = np.hstack((xv_c,yv_c))
                utility = self.util.utility(x_, self.gp, self.clf, 0)
                dims = x[0].shape
                new_utility = np.reshape(utility, dims).T % C-like index orderingd
                heatmap = acq.pcolor(x[0],x[1], new_utility,cmap='seismic')
                plt.colorbar(heatmap)
                acq.plot(xv_c[np.argmax(utility)], yv_c[np.argmax(utility)], '*', markersize=15, 
                         label=u'Next Best Guess', markerfacecolor='gold', markeredgecolor='k', markeredgewidth=1)



                % Plot the clusters
                for i, (mean, covar, color) in enumerate(zip(self.clf.means_, self.clf.covariances_, color_iter))
                        v, w = linalg.eigh(covar)
                        u = w[0] / linalg.norm(w[0])
                        % as the DP will not use every component it has access to
                        % unless it needs it, we shouldn't plot the redundant
                        % components.
                        if not np.any(Y_ == i):
                            continue
                        end
                        acq.autoscale(enable=False)
                        acq.scatter(X_new[Y_ == i, 0], X_new[Y_ == i, 1], 8, color=color, marker='o')
                        % Plot an ellipse to show the Gaussian component
                        angle = np.arctan(u[1] / u[0])
                        angle = 180 * angle / np.pi  % convert to degrees
                        ell = mtlpltpatches.Ellipse(mean, v[0], v[1], 180 + angle, color=color)
                        ell.set_clip_box(acq.bbox)
                        ell.set_alpha(0.5)
                        acq.add_artist(ell)

                end
                new_sigma = np.reshape(sigma, dims).T % C-like index orderingd
                heatmap = var_plot.pcolor(x[0],x[1], new_sigma,cmap='seismic')


                pylab.savefig(str(iterationID) + ".jpg", bbox_inches='tight')
                plt.close("all")
            end
        end
    end
end
              
                