clear all;
filepath = '/home/mcharbonneau/src/learnOptimWBC/matlab/TestResults/results/';
%Retrieve results from simulations
robust_experiments = [4; 8; 11; 14; 17; 21; 22; 28; 29; 30]; %23
performance_experiments = [5; 9; 12; 15; 18; 31; 32; 33; 34; 35];
performanceRobust_experiments = [6; 10; 13; 16; 19; 20; 24; 25; 26; 27];

%% Weights
j_robust = 2;
j_performance = 2;
j_performanceRobust = 2;
for k = 4:35
    if k == 4
        weights_robust(1,:) = [3.14068167739743 2.18029282279953 0 1.0e-10 1.0e-06 1.0e-10];
    elseif k == 5
        weights_performance(1,:) = [1 1 0 0.01 0.001 0.0001];
    elseif k == 6
        weights_performanceRobust(1,:) = [0.521001991358939 3.77297320861414 0 1.0e-10 1.0e-06 1.0e-10];
    elseif k ==7 || k == 23
   
        
    else
        results_file = strcat(num2str(k), '_iCub_standing_sim_1.0/bestAction.mat');
        load( [filepath results_file])
        if find( k == robust_experiments)
            weights_robust(j_robust, :) = bestAction.parameters;
            j_robust = j_robust + 1;
        elseif find( k == performance_experiments)
            weights_performance(j_performance,:) = bestAction.parameters;
            j_performance = j_performance + 1;
        elseif find( k == performanceRobust_experiments)
            weights_performanceRobust(j_performanceRobust,:) = bestAction.parameters;
            j_performanceRobust = j_performanceRobust + 1;
        end
    end
end

%NO DR
for k = 1:10
    results_file = strcat('102_iCub_standing_sim_1.0/', num2str(k), '_of_102_iCub_standing_sim_1.0/bestAction.mat');
    load( [filepath results_file])
    weights_performanceRobustNoDR(k, :) = bestAction.parameters;
end

results.mean_weights_Robust            = mean(weights_robust,1);
results.mean_weights_performance       = mean(weights_performance,1);
results.mean_weights_performanceRobust = mean(weights_performanceRobust,1);
results.mean_weights_performanceRobustNoDR = mean(weights_performanceRobustNoDR,1);
results.std_weights_Robust             = std(weights_robust,1);
results.std_weights_performance        = std(weights_performance,1);
results.std_weights_performanceRobust  = std(weights_performanceRobust,1);
results.std_weights_performanceRobustNoDR = std(weights_performanceRobustNoDR, 1);
% figure(2)
% plot(1, 1, 'bo');
% hold on;
% %for j = [1,2,4,5,6]
% plot(2, weights_robust(1,2)), 'r*';
% plot(2, weights_robust(2,2)), 'g*';
% plot(2, weights_robust(3,2)), 'p*';
% plot(1:6, [1 weights_robust(1,1:2) weights_robust(1,4:6)])
% plot(1:6, [1 weights_robust(2,1:2) weights_robust(2,4:6)])
% plot(1:6, [1 weights_robust(3,1:2) weights_robust(3,4:6)])
% plot(1:6, [1 weights_performance(1,1:2) weights_performance(1,4:6)])
% plot(1:6, [1 weights_performance(2,1:2) weights_performance(2,4:6)])
% plot(1:6, [1 weights_performance(3,1:2) weights_performance(3,4:6)])

disp(results)


