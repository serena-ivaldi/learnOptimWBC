%% for multiple the idea is that the output variable is a vertical matrix
%% this function is extended to manage multiple input for visualization purpose with BayesOpt
function violation=g06Constr1(input,constraints_values)
    violation = (  -( input(:,1) - 5*ones(size(input(:,1))) ).^2 - ( input(:,2) - 5*ones(size(input(:,2))) ).^2 + 100*ones(size(input(:,1)))   );
end