%% TODO extend the oter surrogate to manage a vectorial input
function [ret, x] = poi(obj,x, xi)
    [mean, var] = obj.gp_s{end}.Predict(x);
    % Avoid points with zero variance
    var = max(var, 1e-9 + 0 * var);
    z = (mean - obj.y_max - xi)/sqrt(var);
    ret = cdf(obj.pd,z);
end