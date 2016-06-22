function fit = f241(obj,x)
   input_vector = obj.CreateInputFromParameters(x);
   obj.penalty_handling.EvaluateConstraints(input_vector,1);
   fit = -( -(x(1) + 2*x(2) + 3*x(3) + 4*x(4) + 5*x(5) ) ); 
end