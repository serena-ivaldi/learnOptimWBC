%% %% ( (x(:,1)-5) .^2)./30 - 0.5 < 0.1;
function violation=stuffGPConstr2_1(input,constraints_values)
    violation =  ( (input(:,1)-5) .^2)./30 - 0.5 -0.1 ;
end