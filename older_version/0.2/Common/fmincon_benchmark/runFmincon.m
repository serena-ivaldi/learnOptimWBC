% Now useless
% WILL BE SUPPRESS

function [metric1,metric2,metric3,metric4] = runFmincon(currentProblem,fminconPb,b)
    %Compute fmincon and the metrics for the benchmark
    %written by ugo chervet

% history_fval = [];
% 
% options = optimoptions('fmincon','OutputFcn',@outfun,'Display','iter','Algorithm','sqp');
% options.MaxFunEvals = 500;
% 
% fminconPb.options = options;
% fminconPb.solver = 'fmincon';

% tic;
% [X,FVAL,exitflag,output] = fmincon(fminconPb);
% metric4 =  toc;
% 
% metric1 = fminconPb.J0 - FVAL; %metric 1 = fitness error
% 
% [c, ceq] = currentProblem.contraintesFactice(X); %metric constr violation
% metric2 = sum(abs((c > 0).*c)) + sum(abs((ceq ~= 0).*ceq));
% 
% metric3 = IndetifySteadyState(history_fval,b); %metric 3 = # of steps to steady value

[metric1,metric2,metric3,metric4] = currentProblem.minimize(2,fminconPb.X0,2,2,{fminconPb.LB;fminconPb.UB});


%     function stop = outfun(x,optimValues,state)
%         stop = false;        
%         if strcmp(state, 'iter')
%                 % Concatenate current objective function value with history
%                 history_fval = [history_fval; optimValues.fval];
%         end
%     end

    %find the index after which the values of the vector stay inside (last
    %value of the vecto - treshold) and  (last value of the vecto + treshold)
%     function zzz = IndetifySteadyState(vector,tresh)
%         steady_value = vector(end);
%         for zzz = 1:length(vector)
%             if(abs(steady_value-vector(zzz))<tresh)
%                 break
%             end
%         end
%     end

end
