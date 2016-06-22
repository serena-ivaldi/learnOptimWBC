function fit = f240(obj,x)
   input_vector = obj.CreateInputFromParameters(x);
   obj.penalty_handling.EvaluateConstraints(input_vector,1);
   fit = -( -(x(1) + x(2) + x(3) + x(4) + x(5) ) ); 
end