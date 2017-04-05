%% wrapping function for the constraints to check with fmincon
function [c, ceq]=g06Constr(input)
   constraints_values = 0;
   % Nonlinear inequality constraints
   c1 = g06Constr1(input,constraints_values);
   c2 = g06Constr2(input,constraints_values);
   c = [c1 ;c2];
   % Nonlinear equality constraints
   ceq = [];
end