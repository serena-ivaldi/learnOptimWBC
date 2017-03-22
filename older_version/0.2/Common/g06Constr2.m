%% for multiple input the idea is that the output variable is a vertical matrix
%% this function is extended to manage multiple input for visualization purpose with BayesOpt
function violation=g06Constr2(input,constraints_values)
    violation = ( (input(:,1) - 6*(ones(size(input(:,1)))) ).^2 + ( input(:,2) - 5*(ones(size(input(:,2)))) ).^2 - 82.81*(ones(size(input(:,2))))  );
end