%% TODO extend the oter surrogate to manage a vectorial input
%% to minimize
function [ret, x] = ucb_constr(obj,x)

        [mean, var] = obj.gp_s{end-1}.Predict(x);
        % i have to add a dimension to mean and variance because summation between 
        % array and ndarray is not fun python
        ret =  mean;
end