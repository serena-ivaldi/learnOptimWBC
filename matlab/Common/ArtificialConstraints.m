% here the input is penalties from the constraints_handling obj NoPenalty
% can be used only with NoPenalty obj
function y = ArtificialConstraints(penalties)
    % i check if there are violated constraints
    % in that case pull at all the non violated and sum up the violation
    % otherwise i sum all the constraints that are satisfied
    %% i check for all the constraints that are satisfied 
    index = penalties < 0; % in index: 0 = not satisfied      1 = satisfied
    %% because if the product is zero means that i have violation i sum only the violation 
    %% otherwise i sum all the satisfaction 
    if((prod(index(1,:),2)))
        y = sum(penalties,2);
    else
        penalties(index(1,:)) = 0;
        y = sum(penalties,2);
    end
    y = y(1,:);
end