%% wrapping function for the constraints to check with fmincon
function [c, ceq]=HBConstr(input)
   constraints_values = 0;
   % Nonlinear inequality constraints
   c1 = HBConstr1(input,constraints_values);
   c2 = HBConstr2(input,constraints_values);
   c3 = HBConstr3(input,constraints_values);
   c4 = HBConstr4(input,constraints_values);
   c5 = HBConstr5(input,constraints_values);
   c6 = HBConstr6(input,constraints_values);
   c = [c1; c2; c3; c4; c5; c6];
   % Nonlinear equality constraints
   ceq = [];
end