%% TODO extend the oter surrogate to manage a vectorial input
function [ret, x] = cucb(obj,x)
        [mean, var] = obj.gp_s{end}.Predict(x);
        ret = obj.kappa*sqrt(var)  + mean;
        % constraints computation
        ret2 = ones(size(mean));
        for i=1:obj.n_of_constraints
            [mean, var] = obj.gp_s{i}.Predict(x);
            ret2 = ret2.*(normcdf(0,mean,sqrt(var)));
        end
        ret = ret.*ret2;
end