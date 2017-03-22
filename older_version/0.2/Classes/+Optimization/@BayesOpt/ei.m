%% TODO extend the oter surrogate to manage a vectorial input
function [ret, x] = ei(obj,x)
        [mean, var] = obj.gp_s{end}.Predict(x);
        y_max = obj.y_max*ones(size(mean));
        XI = obj.xi*ones(size(mean));
        % Avoid points with zero variance
        var = max(var, 1e-9 + 0 * var);
        z = (mean - y_max - XI)./sqrt(var);
        ret = (mean - y_max - XI) .* cdf(obj.pd,z) + sqrt(var) .* pdf(obj.pd,z);
end