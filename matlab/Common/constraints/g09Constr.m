%% wrapping function for the constraints to check with fmincon
function [c, ceq]=g09Constr(input)
   constraints_values = 0;
   % Nonlinear inequality constraints
   c1 = g09Constr1(input,constraints_values);
   c2 = g09Constr2(input,constraints_values);
   c3 = g09Constr3(input,constraints_values);
   c4 = g09Constr4(input,constraints_values);
   c = [c1; c2; c3; c4];
   % Nonlinear equality constraints
   ceq = [];
end