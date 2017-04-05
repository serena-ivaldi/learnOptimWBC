function violation=g07Constr4(input,constraints_values)
    violation =  - 3*input(1)  + 6*input(2)  + 12*(input(9) - 8 )^(2) - 7*input(10);
end