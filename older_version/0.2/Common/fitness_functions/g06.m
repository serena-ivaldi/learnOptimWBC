%% in this case output is the candidate because Im dealing with simple optimization problem
%% for benchmarking purpose
%% for multiple input the idea is that the output variable is a vertical matrix
%% this function is extended to manage multiple input for visualization purpose with BayesOpt
function fit = g06(obj,output)
   if(~isempty(obj))
       input_vector = obj.CreateInputFromParameters(output);
       obj.penalty_handling.EvaluateConstraints(input_vector,1);
   end
   fit = -(  ( output(:,1) - 10*(ones(size(output(:,1)))) ).^3 + ( output(:,2) - 20*(ones(size(output(:,2)))) ).^3  ); 
end