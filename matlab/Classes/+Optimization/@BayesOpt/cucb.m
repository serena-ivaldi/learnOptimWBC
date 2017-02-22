%% this function is a big mistake because is not gonna work due to the fact that the probability of constraints satisfaction acts as a penalty that clamp all the region 
%% that violates constraints to zero. it means that if i have to minimize a function where the minima are bigger than zero or i have to maximize a function where the maxima are below 
%% zero suddenly the constrained became the best solution so we can use the probability of constraints on ly with function that are known ahead: 
%% for example
%% probability of improvement (max with positive>0 maxima)
%% constraints violations     (max with positive>0 maxima)
%% expected improvements      (max with positive>0 maxima)
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