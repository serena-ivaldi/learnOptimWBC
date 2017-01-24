%% in this case output is the candidate because Im dealing with simple optimization problem
%% for benchmarking purpose
function fit = rosenbrock(obj,output)
   input_vector = obj.CreateInputFromParameters(output);
   % i do not have any constraints
   %obj.penalty_handling.EvaluateConstraints(input_vector,1);
   fit = -( 100*(output(2)-output(1)^(2))^(2) +(output(1)-1)^(2) ); 
end