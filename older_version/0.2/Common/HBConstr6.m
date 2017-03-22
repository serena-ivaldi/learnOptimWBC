% h1 = 85.334407 + 0.0056858*input(2)*input(5) + 0.00026*input(1)*input(4) - 0.0022053*input(3)*input(5);
% h2 = 80.51249 + 0.0071317*input(2)*input(5) + 0.0029955*input(1)*input(2) + 0.0021813*input(3)^(2);
% h3 = 9.300961 + 0.0047026*input(3)*input(5) + 0.0012547*input(1)*input(3) + 0.0019085*input(3)*input(4);
function violation=HBConstr6(input,constraints_values)
    h3 = 9.300961 + 0.0047026*input(3)*input(5) + 0.0012547*input(1)*input(3) + 0.0019085*input(3)*input(4);
    violation =  h3 -25 ;
end