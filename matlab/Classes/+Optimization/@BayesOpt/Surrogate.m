function ret = Surrogate(self, x, kind, kappa, xi)
        if strcmp(kind,'ucb')
            ret = ucb(self,x, gp, clf, kappa, risk, base);
        end
        if strcmp(kind,'ei')
            ret = ei(self,x, xi);
        end
        if strcmp(kind,'poi')
            ret = poi(self,x, xi);
        end
end


function ret = ucb(obj,x, kappa)
        [mean, var] = obj.gp_s{end}.predict(x);
        % i have to add a dimension to mean and variance because summation between 
        % array and ndarray is not fun python
        ret = sqrt(var)  + mean;
end

function ret = ei(obj,x, xi)
        [mean, var] = obj.gp_s{end}.predict(x);

        % Avoid points with zero variance
        var = max(var, 1e-9 + 0 * var);

        z = (mean - self.y_max - xi)/sqrt(var);
        ret = (mean - self.y_max - xi) * norm.cdf(z) + sqrt(var) * norm.pdf(z);
end

function ret = poi(obj,x, xi)
    [mean, var] = obj.gp_s{end}.predict(x);
    % Avoid points with zero variance
    var = max(var, 1e-9 + 0 * var);

    z = (mean - self.y_max - xi)/np.sqrt(var);
    ret = norm.cdf(z);
end
    