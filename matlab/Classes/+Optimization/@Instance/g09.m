function fit = g09(obj,x)
   input_vector = obj.CreateInputFromParameters(x);
   obj.penalty_handling.EvaluateConstraints(input_vector,1);
   fit = -( (x(1)-10)^(2) + 5*(x(2) - 12 )^(2) + x(3)^(4) +3*(x(4)-11)^(2)...
           +10*x(5)^(6) +7*x(6)^(2) +x(7)^(4) -4*x(6)*x(7) -10*x(6) -8*x(7)); 
end