function [mean_performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = LearnCMAES(obj,settings)

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
p_sigma = zeros(nIterations, n);
p_c = zeros(nIterations, n);


mean(1, :) = action;

range = maxAction - minAction;

stdDev = range;

sigma = zeros(nIterations,1);
sigma(1) = explorationRate;
C{1} = diag(stdDev.^2);

if isfield(settings,'nOffsprings')
    lambda = settings.nOffsprings;
else
    lambda = round(4 + 3 * log(n));
end
mu = round(lambda / 2);

offsprings = zeros(lambda, n);
performances = zeros(lambda, 1);

w = zeros(mu,1);
for l = 1:mu
    w(l) = log(mu + 1) - log(l);
end
w = w / sum(w);
ueff = sum(w.^2)^-1;
cc = 4 / (n + 4);
ccov = 2 / (n + sqrt(2))^2;

chi_n = sqrt(2) * gamma((n + 1) /2) / gamma(n/2);

c_sigma = (ueff + 2) / (n + ueff  + 3);
d_sigma = 1 + 2 * max([0, sqrt((ueff - 1)/(n-1))]) + c_sigma;

mean_performances = zeros(nIterations,1);
policies = zeros(nIterations*lambda, n);
BestActionPerEachGenPolicy = zeros(nIterations, n);
BestActionPerEachGenFitness= zeros(nIterations, 1);
costs = zeros(1,nIterations*lambda);
succeeded = zeros(1,nIterations*lambda);
policyId = 1;

if ~isfield(settings,'allowEvalMultiple')
    settings.allowEvalMultiple = 0;
else
    settings.actionMultiple = repmat(action,lambda,1);
    if isfield(settings, 'activeIndices')
        settings.activeIndicesMultiple = repmat(settings.activeIndices, lambda,1);
    end
end

fnForwardModel = @(obj_,actionLearn_,curr_candidate_,isMean_)TransAction(obj_,actionLearn_,curr_candidate_,isMean_, settings);
% first mean
[mean_performances(1), succeeded(1), data2save] = fnForwardModel(obj,mean(1, :),-1,1);
policies(policyId,:) = mean(1, :);
costs(policyId) = -mean_performances(1);
policyId = policyId + 1;
G_data2save.performance(1,1) = data2save.performance;

fprintf('Mean %d: %e %d\n', 1 , mean_performances(1), succeeded(1));
for k = 1:(nIterations - 1)
    
    if settings.plotState
        fprintf('iteration %d\n',k);
    end
    %create offsprings
    disp('create offsprings')
    for l = 1:lambda 

        offsprings(l, :) = mean(k,:) + sigma(k) * mvnrnd(zeros(1, n), C{k});
        
        offsprings(l, offsprings(l,:) > maxAction) = maxAction(offsprings(l,:) > maxAction);
        offsprings(l, offsprings(l,:) < minAction) = minAction(offsprings(l,:) < minAction);
    end
    
    %% DEBUG
    if(isnan(offsprings))
       disp('stop it, offsprings is nan');
    end
    %%
    
    
    %evaluate offsprings
    disp('evaluate offsprings')
    if settings.allowEvalMultiple > 0
        performances = fnForwardModel(obj,offsprings,0, 0);
%         keyboard %check correctness of ids
        ids = policyId:policyId+lambda-1;
        policies(ids,:) = offsprings;
        costs(ids) = -performances;
        policyId = policyId + lambda;
    else
        %par
        for l = 1:lambda 
            [performances(l) succeeded(policyId)] = fnForwardModel(obj,offsprings(l, :),l, 0);
            if settings.plotState
                fprintf('Offspring %d : %e %d\n', l, performances(l), succeeded(policyId));
            end
            policies(policyId,:) = offsprings(l, :);
            costs(policyId) = -performances(l);
            policyId = policyId + 1;
        end
    end
        
    %% added part to manage constraints
    if(obj.constraints)
         [costs,performances]=obj.penalty_handling.FitnessWithPenalty(policyId,costs,performances,k+1);
    end
     
    %%
    [sortVal, sortInd] = sort(performances, 'descend');
    
    for l = 1:mu
        index = sortInd(l);
        mean(k + 1, :) = mean(k + 1, :) + w(l) * offsprings(index, :);
    end
    % end generation mean
    [mean_performances(k + 1), succeeded(policyId), data2save] = fnForwardModel(obj,mean(k + 1, :),-(k+1),1);
    policies(policyId,:) = mean(k + 1, :);
    costs(policyId) = -mean_performances(k + 1);
    policyId = policyId + 1;
    G_data2save.performance(k + 1,1) = data2save.performance;
    
    bestAction.hist(k).performance = mean_performances(k + 1);
    bestAction.hist(k).listperformance = performances;
    bestAction.hist(k).parameters = mean(k + 1, :);
    bestAction.hist(k).variance =  var(offsprings);
    
    % im building the data set to cluster it in a second phase
    BestActionPerEachGenPolicy(k,:)= offsprings(sortInd(1),:);
    BestActionPerEachGenFitness(k,1) = performances(sortInd(1));

    fprintf('Mean %d: %e %d\n', k + 1, mean_performances(k + 1), succeeded(policyId-1));

    
    p_c(k + 1,:) = (1 - cc) * p_c(k, :) + sqrt(cc * (2-cc) * ueff) *(mean(k + 1, :) - mean(k,:)) / sigma(k);
    C{k + 1} = (1 - ccov) * C{k} + ccov * (1 / ueff * p_c(k + 1,:)' * p_c(k + 1,:));
    
    for l = 1:mu
        index = sortInd(l);
        C{k + 1} = C{k + 1} + ccov * (1 - 1 / ueff) * w(l) * (offsprings(index, :) - mean(k,:))' * (offsprings(index, :) - mean(k,:)) / sigma(k)^2;
    end

    [B,D] = eig(C{k});
    
    p_sigma(k + 1, :) = (1 - c_sigma) * p_sigma(k,:) + (sqrt(c_sigma * (2 - c_sigma) * ueff) *(B * D ^ (-0.5) * B') * (mean(k + 1, :) - mean(k,:))' / sigma(k))';
    sigma(k + 1) = sigma(k) * exp(c_sigma/d_sigma * (norm(p_sigma(k+1,:)) / chi_n - 1));    
    if(isnan(sigma(k + 1)))
       sigma(k + 1) = sigma(k);
    end
end

policies = policies(1:policyId-1,:);
costs = costs(1:policyId-1);
succeeded = succeeded(1:policyId-1);


BestActionPerEachGen.policy = BestActionPerEachGenPolicy;
BestActionPerEachGen.fitness = BestActionPerEachGenFitness;
G_data2save.C = C;

%bestAction.parameters = mean(end, :);
%bestAction.performance = fnForwardModel(bestAction.parameters);
[tmp, id] = min(costs);
bestAction.parameters = policies(id,:);
bestAction.performance = -costs(id);

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