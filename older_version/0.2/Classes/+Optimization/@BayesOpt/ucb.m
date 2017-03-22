%% TODO extend the oter surrogate to manage a vectorial input
function [ret, x] = ucb(obj,x)
        [mean, var] = obj.gp_s{end}.Predict(x);
        ret = obj.kappa*sqrt(var)  + mean;
end