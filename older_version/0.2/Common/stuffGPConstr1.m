%% wrapping function for the constraints to check with fmincon
% i have to change the first meber sign tto rewrite the constraints as 
%  as a less than inequality
function violation=stuffGPConstr1(input,constraints_values)
    violation =  ((input(:,1)-5) .^2 + (input(:,2)-5).^2 -1)/30;
end