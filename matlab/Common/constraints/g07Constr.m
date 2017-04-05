%% wrapping function for the constraints to check with fmincon
function [c, ceq]=g07Constr(input)
   constraints_values = 0;
   % Nonlinear inequality constraints
   c1 = g07Constr1(input,constraints_values);
   c2 = g07Constr2(input,constraints_values);
   c3 = g07Constr3(input,constraints_values);
   c4 = g07Constr4(input,constraints_values);
   c5 = g07Constr5(input,constraints_values);
   c6 = g07Constr6(input,constraints_values);
   c7 = g07Constr7(input,constraints_values);
   c8 = g07Constr8(input,constraints_values);
   c = [c1; c2; c3; c4; c5; c6; c7; c8];
   % Nonlinear equality constraints
   ceq = [];
end