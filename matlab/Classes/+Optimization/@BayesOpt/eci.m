%% class BO
% eci =  expcted constrained improvement 
% to maximize
function [ret, x]= eci(obj,x, xi)       
        % ei computation
        [mean, var] = obj.gp_s{end}.Predict(x);
        y_max = obj.y_max*ones(size(mean));
        XI = xi*ones(size(mean));
        % Avoid points with zero variance
        var = max(var, 1e-9 + 0 * var);
        z = (mean - y_max - XI)./sqrt(var);
        ret1 = (mean - y_max - XI) .* cdf(obj.pd,z) + sqrt(var) .* pdf(obj.pd,z);
        % constraints computation
        ret2 = ones(size(mean));
        for i=1:obj.n_of_constraints
            [mean, var] = obj.gp_s{i}.Predict(x);
            %% TODEBUG
            %probability = (normcdf(0,mean,sqrt(var)));
            ret2 = ret2.*(normcdf(0,mean,sqrt(var)));
        end
        ret = ret2.*ret1;       
end