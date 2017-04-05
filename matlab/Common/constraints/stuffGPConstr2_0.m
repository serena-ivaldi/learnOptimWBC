%% ( (x(:,1)-5) .^2)./30 - 0.5 > -10;
function violation=stuffGPConstr2_0(input,constraints_values)
    violation =  -( (input(:,1)-5) .^2)./30 + 0.5 - 10 ;
end