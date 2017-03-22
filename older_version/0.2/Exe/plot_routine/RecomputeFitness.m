cut = true;
cut_point = 53;


% plot of fitnes function
evolutions = size(bestAction.hist,2);
% remove failed mean and failed variance
% -10000000 is the penalty that is applied when i have a failure 
index = 1;
evo = 1;
mean = 0;
for ww =1:evolutions
    % i discard the evolutions with failure final mean
    if(bestAction.hist(1,ww).performance>-1)
        listperformance = bestAction.hist(1,ww).listperformance;
        %remove all the failure from the computation of the variance
        listperformance = listperformance(listperformance~=-1);
        mean(index) = bestAction.hist(1,ww).performance;
        evo(index) = ww;
        index = index + 1; 
    end
    
end


% cut data from a predefined x value
if(cut)
    cut_ind = find(evo==cut_point);
    mean = mean(cut_ind:end);
    evo = evo(cut_ind:end);
end

evo  = evo - (evo(1));




figure
plot(evo',mean');
grid on