%% TODO extend the oter surrogate to manage a vectorial input
function [ret, x] = ucb(obj,x)
        [mean, var] = obj.gp_s{end}.Predict(x);
        % i have to add a dimension to mean and variance because summation between 
        % array and ndarray is not fun python
        ret = obj.kappa*sqrt(var)  + mean;
end