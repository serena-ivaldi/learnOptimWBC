%% wrapping function for the constraints to check with fmincon
function [c, ceq]=g10Constr(input)
   constraints_values = 0;
   % Nonlinear inequality constraints
   c1 = g10Constr1(input,constraints_values);
   c2 = g10Constr2(input,constraints_values);
   c3 = g10Constr3(input,constraints_values);
   c4 = g10Constr4(input,constraints_values);
   c5 = g10Constr5(input,constraints_values);
   c6 = g10Constr6(input,constraints_values);
   c = [c1; c2; c3; c4; c5; c6;];
   % Nonlinear equality constraints
   ceq = [];
end