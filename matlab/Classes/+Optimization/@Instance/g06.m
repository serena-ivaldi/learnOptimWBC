%% in this case output is the candidate because Im dealing with simple optimization problem
%% for benchmarking purpose
function fit = g06(obj,output)
input_vector = obj.CreateInputFromParameters(output);
obj.penalty_handling.EvaluateConstraints(input_vector,1);
fit = -((output(1) - 10)^3 + (output(2) - 20)^3); 
end