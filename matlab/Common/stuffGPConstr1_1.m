%% wrapping function for the constraints to check with fmincon
% this constraints need to be lower than 0.8 ---> move 0.8 on the left
% member with the opposite sign
function violation=stuffGPConstr1_1(input,constraints_values)
    violation =  ((input(:,1)-5) .^2 + (input(:,2)-5).^2 - 1)/30 -0.8;
end