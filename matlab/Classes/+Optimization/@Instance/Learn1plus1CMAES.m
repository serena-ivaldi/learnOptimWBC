function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = Learn1plus1CMAES(obj,settings)

nIterations = settings.nIterations;
explorationRate = settings.explorationRate;

if isfield(settings, 'activeIndices')
    action = settings.action(settings.activeIndices);
    minAction = settings.minAction(settings.activeIndices);
    maxAction = settings.maxAction(settings.activeIndices);
else
    action = settings.action;
    minAction = settings.minAction;
    maxAction = settings.maxAction;
end

n = size(action,2);
mean = zeros(nIterations, n);
mean(1, :) = action;
range = maxAction - minAction;
stdDev = range;
sigma = zeros(nIterations,1);
sigma(1) = explorationRate;
C = diag(stdDev.^2);
A{1} = chol(C); 

performances = zeros(1,nIterations);  % row vector
costs = zeros(1,nIterations);      
succeeded = zeros(1,nIterations);

n_constraints = obj.penalty_handling.n_constraint;

% initialization
for j = 1 : n_constraints
   v(j,:) = zeros(1,n);    
end
V{1} = v;   %cell vector of matrix where each v is row
s = zeros(nIterations,n); 
d = 1 + n/2;
c =  2/ (n + 2);
c_p = 1/12;
P_succ = 0;   % im not sure
P_target = 2/12;
c_cov_plus = 2 /(n^(2) + 6);
c_cov_minus = 0.4/(n^(1.6)+1);
c_c = 1/(n+2);
beta = 0.1/(n+2);


fnForwardModel = @(obj_,actionLearn_,curr_candidate_,isMean_)TransAction(obj_,actionLearn_,curr_candidate_,isMean_, settings);


%% Initialization
%% at the beginning i need to provide a solution that is feasible 
bounds = [minAction',maxAction'];
options.testflag  = 0; options.showits   = 1;  options.maxevals = 25000;
options.tol       = 0.001; options.maxits = 10000000000;
Problem.f = 'FeasibleSolutionObj';
% Call DIRECT
% [fmin,xmin,hist] = Direct(Problem,bounds,options,obj);
% mean(1, :) = xmin';
% if(fmin > 0)
%    x0 = (maxAction-minAction).*rand(1,n) + minAction;
%    options = optimoptions(@fmincon,'Algorithm','sqp');
%    func_to_optimize = str2func(Problem.f);
%    func_to_optimize = @(x)func_to_optimize(x,obj);
%    [new_xmin,new_fmin] = fmincon(func_to_optimize,x0,[],[],[],[],bounds(:,1)',bounds(:,2)',[],options); 
%    mean(1, :) = new_xmin';
% end
[performances(1), succeeded(1), data2save] = fnForwardModel(obj,mean(1, :),1,1);
costs(1) = - performances(1);

fprintf('Mean %d: %e %d\n', 1 , performances(1), succeeded(1));
for k = 1:(nIterations - 1)
    if settings.plotState
        fprintf('iteration %d\n',k);
    end
    %create offsprings
    disp('create offsprings')
    %C = A{k}'*A{k};
    %z = mvnrnd(zeros(1, n), C);
    %offsprings = mean(k,:) + sigma(k) *z;
    z =  mvnrnd(zeros(1, n), eye(n));
    offsprings = mean(k,:) + sigma(k) *(A{k} * z')';
    % DEBUG
    if(isnan(offsprings))
       disp('somethings wrong took place');
    end
    %
    offsprings(1, offsprings(1,:) > maxAction) = maxAction(offsprings(1,:) > maxAction);
    offsprings(1, offsprings(1,:) < minAction) = minAction(offsprings(1,:) < minAction);
    
    %% DEBUG
    if(isnan(offsprings))
       disp('stop it, offsprings is nan');
    end
    %%    
    %evaluate offsprings
    disp('evaluate offsprings')
    [performances_new succeeded(k)] = fnForwardModel(obj,offsprings,1, 1); % compute fitness    
    %% added part to manage constraints
    if(obj.constraints)      
        constraints = obj.penalty_handling.penalties(1,:)>0; % vector of index of the violated constrained
        violated_constrained = find(constraints);
    end 
    %%  update   
    if(~isempty(violated_constrained)) % some constraints are violated
       v = V{k};
       for j = violated_constrained
         v(j,:) = (1-c_c)*V{k}(j,:) + c_c*(A{k}*z')';                                           %only if the constraints is violated udpate exponentially fading record vj 
       end
       V{k+1} = v;
       index = 1;
       %v(j,:); % to fix no commit without fixing
       for j = violated_constrained
         w(index,:) = (A{k}^(-1)*v(j,:)')';
         index = index + 1;
       end
       mean(k+1,:) = mean(k,:);                                                                                                                             % no update mean
       performances(k+1) = performances(k);
       costs(k+1) = -performances(k);
       s(k+1,:) = s(k,:);                                                                                                                                      % no update s
       value = zeros(size(A{1}));
       index = 1;
       for j = violated_constrained
         value = value +  (v(j,:)'*w(index,:))/(w(index,:)*w(index,:)');
         index = index + 1;
       end
       A{k+1} = A{k} - (beta)/length(violated_constrained) * value;                                                             % update A if constraint violation is true
       sigma(k+1) = sigma(k);                                                                                                                              % no update sigma
    else % all the constraints are satisfacted
       if(performances_new > performances(k))
          P_succ = (1-c_p)*P_succ + c_p; 
       else
          P_succ = (1-c_p)*P_succ; 
       end
       sigma(k+1) = sigma(k)*exp( (1/d) * (P_succ - P_target) / (1-P_target) );                                                                                % update sigma
       if(performances_new > performances(k)) % perfomance is better
          mean(k+1,:) = offsprings;                                                                                                                             % update mean
          performances(k+1) = performances_new;
          costs(k+1) = -performances_new;
          s(k+1,:) = (1-c)*s(k,:) + sqrt(c*(2-c))*(A{k}*z')';                                       %only if the constraints are not violated udpate exponentially fading record s
          w = (A{k}^(-1)*s(k+1,:)')'; 
          A{k+1} = sqrt(1 - c_cov_plus)*A{k} + ( sqrt(1-c_cov_plus)/norm(w)^2 )*(sqrt(1 + (c_cov_plus*norm(w)^2)/(1-c_cov_plus) ) - 1 )*s(k+1,:)'*w; % update A if perfor_new > perf(k)
          V{k+1} = V{k};                                                                                                                                       % no update v
       else % perfomance is worse
          mean(k+1,:) = mean(k,:);                                                                                                                            % no update mean
          performances(k+1) = performances(k);                                                                                                               
          costs(k+1) = -performances(k);
          s(k+1,:) = s(k,:);                                                                                                                                    % no update s
          V{k+1} = V{k};                                                                                                                                        % no update v
          if(k>5)   
             if(performances_new > performances(k-5)) % perfomance is worse but better than the last fifth predecessor
                A{k+1} = sqrt(1 + c_cov_minus)*A{k} + ( sqrt(1 + c_cov_minus)/norm(z)^2 )*( sqrt(1 - (c_cov_minus*norm(z)^2)/(1 + c_cov_minus)) - 1 )*A{k}*(z'*z); % update A if perf_new>perf(k-5)
             else
                A{k+1} = A{k};                                                                                                                                     % A no update
             end
          else
             A{k+1} = A{k};         
          end
       end
    end
    bestAction.hist(k).performance = performances(k + 1);
    bestAction.hist(k).parameters = mean(k + 1, :);
    
    fprintf('perfomance %d: %e %d\n', k + 1, performances(k + 1), succeeded(k));

end
G_data2save.A = A;
G_data2save.C = [];
G_data2save.performance = [0];

BestActionPerEachGen = [];
policies = [];

bestAction.parameters = mean(end,:);
bestAction.performance = -costs(:,end);
bestAction.listperformance = performances;
end

function [performance, succeeded, data2save ] = TransAction(obj_,actionLearn, curr_candidate,isMean, settings)
if isfield(settings, 'activeIndices')
    if size(actionLearn,1) < 2
        actionFull = settings.action;
        actionFull(settings.activeIndices) = actionLearn;
    else
        actionFull = settings.actionMultiple;
        actionFull(settings.activeIndicesMultiple) = actionLearn;
    end
else
    actionFull = actionLearn;
end
    [performance, succeeded, data2save] = settings.fnForwardModel(obj_, actionFull, curr_candidate ,isMean);

end
