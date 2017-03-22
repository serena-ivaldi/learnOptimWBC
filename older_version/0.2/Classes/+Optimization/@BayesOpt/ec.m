%% class BO
%% we need to maximize this function 
function [ret, x] = ec(obj,x)
                
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
        ret2 = ones(length(x),1);
        for i=1:obj.n_of_constraints
            [mean, var] = obj.gp_s{i}.Predict(x);
            %% TODEBUG
            %probability = (normcdf(0,mean,sqrt(var)));
            ret2 = ret2.*(normcdf(0,mean,sqrt(var)));
        end
        ret = ret2;
        
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