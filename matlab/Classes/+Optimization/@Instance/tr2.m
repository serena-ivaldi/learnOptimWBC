function fit = tr2(obj,output)
   input_vector = obj.CreateInputFromParameters(output);
   obj.penalty_handling.EvaluateConstraints(input_vector,1);
   fit = -( (output(1))^2 + (output(2))^2 ); 
end