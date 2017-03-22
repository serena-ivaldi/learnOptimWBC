%% class BO pcs = probability of constraints satisfaction
% to maximize
function [ret, x] = pcs_constr(obj,x)
    siz = size(x);
    len = siz(siz~=obj.dim);
    ret = ones(len,1);
    for i=1:obj.n_of_constraints
        [mean_, var_] = obj.gp_s{i}.Predict(x);
        %% TODEBUG
        %probability = (normcdf(0,mean,sqrt(var)));
        ret = ret.*(normcdf(0,mean_,sqrt(var_)));
    end
end