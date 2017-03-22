%% class BO pcs = probability of constraints satisfaction
% to maximize
% mcd = maximum constrained distance i try to maximize the dsitance from
% the particles already deployed inside the free region
% because the first varargin is reserved to the function handle for the
% zoom i need to use the second slot to pass to the function the parameters
% that i need 
function [ret1, x] = mcd_constr(obj,x,particles_pos)
    siz = size(x);
    len = siz(siz~=obj.dim);
    if isempty(len)
        len = length(x);
    end
    % this is not a big deal because im gonna trigger this cicle only when
    % i want to plot the function (just for debugging)
    siz1 = size(particles_pos);
    len1 = siz1(siz1~=obj.dim);
    if isempty(len1)
        len1 = length(particles_pos);
    end
    for jj = 1:len
        % compute distance from particles
        extended_x = repmat(x(jj,:),len1,1); 
        dist_sum(jj,1) = sum(sqrt(sum((extended_x - particles_pos).^2,2)),1)/len1; % average distance from all the particle
    end
    % compute constrained space
    ret = ones(len,1);
    for i=1:obj.n_of_constraints
        [mean_, var_] = obj.gp_s{i}.Predict(x);
        %% TODEBUG
        %probability = (normcdf(0,mean,sqrt(var)));
        ret = ret.*(normcdf(0,mean_,sqrt(var_)));
    end
    
    ret1 = dist_sum .* ret;
    
end