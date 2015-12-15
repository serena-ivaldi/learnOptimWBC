%% wrapping function for the constraints to check with fmincon
function [c, ceq]=tr2Constr(input)
   constraints_values = 0;
   % Nonlinear inequality constraints
   c1 = tr2Constr1(input,constraints_values);
   c = [c1];
   % Nonlinear equality constraints
   ceq = [];
end