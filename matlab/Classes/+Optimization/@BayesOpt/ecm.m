%% class BO (expected mean of constraints)
%% it does nor work for multiple input
%% to minimize
function [ret, x] = ecm(obj,x)
                
        %% TODEBUG
%         if(size(x,1)==1)
%             x_crash = [ 2.2199, 5.9993];
%             dist = norm(x - x_crash);
%             tolerated_distance = 0.0001;
%             if(dist < tolerated_distance)
%                 disp('wake up!')
%             end
%      
        % constraints computation
        ret = ones(1,1);
        for i=1:obj.n_of_constraints
            [mean, var] = obj.gp_s{i}.Predict(x);
            %% TODEBUG
            %probability = (normcdf(0,mean,sqrt(var)));
            if(mean > 0)
                mean = 0;
            end
             ret = ret*mean;
        end
        
        
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