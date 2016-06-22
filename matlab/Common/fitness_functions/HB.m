function fit = HB(obj,x)
   input_vector = obj.CreateInputFromParameters(x);
   obj.penalty_handling.EvaluateConstraints(input_vector,1);
   fit = -(5.3578547*x(3)^(2) + 0.8356891*x(1)*x(5) + 37.293239*x(1) -40792.141 ); 
end