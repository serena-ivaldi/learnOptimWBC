function violation=g06Constr1(input,constraints_values)
    violation = -((input(1) - 5)^2 -(input(2) - 5)^2 + 100);
end