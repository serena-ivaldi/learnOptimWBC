%Script computing the perfomances of fmincon on 3 known problems
%written by ugo chervet

clear all
close all

nbrep = 50; %50
b = 50; %radius of the topologic ball for the metric3 (express as a % of difference from the optimal

%problem_name = {'g07'};
problem_name = {'g07','g09','HB'};
metric1 = [];
metric2 = [];
metric3 = [];
metric4 = [];


for i = 1:length(problem_name)
    fminconPb.name = problem_name{i};
    
    currentProblem = ObjProblem(fminconPb.name);
    
    fminconPb.J0 = currentProblem.J0;
    fminconPb.objective = @currentProblem.computFit;
    fminconPb.LB = currentProblem.LB;
    fminconPb.UB = currentProblem.UB;
    %generation of a random starting point inside the limit boundaries
    fminconPb.NONLCON = @currentProblem.contraintesFactice;
    
    for iter=1:nbrep
        currentProblem.randStartPoint();
        fminconPb.X0 = currentProblem.X0;
        [m1(iter),m2(iter),m3(iter),m4(iter)] = currentProblem.minimize(length(fminconPb.X0),fminconPb.X0,[],[],{fminconPb.LB;fminconPb.UB});
    end
    metric1 =  [metric1,  m1]; %metric 1 = fitness error
    metric2 =  [metric2,  m2]; %metric 2 = constraints violation
    metric3 =  [metric3,  m3]; %metric 3 = # of steps to steady value
    metric4 =  [metric4,  m4]; %metric 4 = compuating duration
end

%matrice to automaticly adpat the box plot to the number of problems
A=[];
for i=1:length(problem_name)
    A = [A; i*ones(nbrep,1)];
end

figure(1);
colors = [1 0 0; 0 0 1; 0 0.5 0];
subplot(1,4,1)
boxplot(metric1', A, 'Colors',colors);
title('Fitness error','FontSize',10);
subplot(1,4,2)
boxplot(metric2', A, 'Colors',colors);
title('Constraints violation','FontSize',10);
subplot(1,4,3)
boxplot(metric3', A, 'Colors',colors);
title('# of steps to steady value','FontSize',10);
subplot(1,4,4)
boxplot(metric4', A, 'Colors',colors);
title('Times','FontSize',10);
