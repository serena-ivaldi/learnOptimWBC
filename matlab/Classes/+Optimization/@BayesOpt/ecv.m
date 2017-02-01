%% class BO
function ret = ecv(obj,x)
                
        %% TODEBUG
%         if(size(x,1)==1)
%             x_crash = [ 2.2199, 5.9993];
%             dist = norm(x - x_crash);
%             tolerated_distance = 0.0001;
%             if(dist < tolerated_distance)
%                 disp('wake up!')
%             end
%         end       
        % ev computation
        [mean, var] = obj.gp_s{end}.Predict(x);
        % Avoid points with zero variance
        var = max(var, 1e-9 + 0 * var);
        % constraints computation
        ret2 = ones(size(mean));
        for i=1:obj.n_of_constraints
            [mean, var] = obj.gp_s{i}.Predict(x);
            %% TODEBUG
            %probability = (normcdf(0,mean,sqrt(var)));
            ret2 = ret2.*(normcdf(0,mean,sqrt(var)));
        end
        ret = ret2.*var;
        
        %% TODEBUG
%         if(size(x,1)==1)
%             disp('check for incosistent value for surrogate function')
%             x
%             mean
%             var
%             ret2
%             ret
%         end
        
end