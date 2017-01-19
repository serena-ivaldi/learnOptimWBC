%% in order to maximize with a minimization a have to put a minus in front
%% of all the surrogates that are natively built as a maximization problem
function ret = Surrogate(self, x, kind, kappa, xi)
        if strcmp(kind,'ucb')
            ret = -ucb(self,x, kappa);
        end
        if strcmp(kind,'ei')
            ret = -ei(self,x, xi);
        end
        if strcmp(kind,'poi')
            ret = -poi(self,x, xi);
        end
        if strcmp(kind,'eci')
            ret = -eci(self,x, xi);
        end
end


function ret = ucb(obj,x, kappa)
        [mean, var] = obj.gp_s{end}.Predict(x);
        % i have to add a dimension to mean and variance because summation between 
        % array and ndarray is not fun python
        ret = kappa*sqrt(var)  + mean;
end

function ret = ei(obj,x, xi)
        [mean, var] = obj.gp_s{end}.Predict(x);

        % Avoid points with zero variance
        var = max(var, 1e-9 + 0 * var);
        z = (mean - obj.y_max - xi)/sqrt(var);
        ret = (mean - obj.y_max - xi) * cdf(obj.pd,z) + sqrt(var) * pdf(obj.pd,z);
end

function ret = poi(obj,x, xi)
    [mean, var] = obj.gp_s{end}.Predict(x);
    % Avoid points with zero variance
    var = max(var, 1e-9 + 0 * var);
    z = (mean - obj.y_max - xi)/sqrt(var);
    ret = cdf(obj.pd,z);
end


function ret = eci(obj,x, xi)
        
        % ei computation
        [mean, var] = obj.gp_s{end}.Predict(x);
        % Avoid points with zero variance
        var = max(var, 1e-9 + 0 * var);
        z = (mean - obj.y_max - xi)/sqrt(var);
        ret1 = (mean - obj.y_max - xi) * cdf(obj.pd,z) + sqrt(var) * pdf(obj.pd,z);
        % constraints computation
        ret2 = 1;
        for i=1:obj.n_of_constraints
            [mean, var] = obj.gp_s{i}.Predict(x);
            %% TODEBUG
            probability = (normcdf(0,mean,sqrt(var)));
            ret2 = ret2*(normcdf(0,mean,sqrt(var)));
        end
        ret = ret2*ret1;
        
end
    