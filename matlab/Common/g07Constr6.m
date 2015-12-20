function violation=g07Constr6(input,constraints_values)
    violation =  +input(1)^(2)  +2*(input(2) -2)^(2)  -2*input(1)*input(2) +14*input(5) -6*input(6);
end