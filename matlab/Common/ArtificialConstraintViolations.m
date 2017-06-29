% here the input is constraints_violation so it means that i have to 
% to do other analisys to the input in order to obtain the value final
% value for each constraints. (i have to do a ComputePenalties passage in the same way as is done in noPenalty class to
% obtain the penalties matrix) (refer to the penalty objects for understanding the meaning of each variables)
% this function is a bit more general than ArtificialConstraints it can be
% used for all the penalty_handling object
function [y,penalties,violation_index] = ArtificialConstraintViolations(constraints_violation,n_constraint)
    % i check if there are violated constraints
    % in that case pull at all the non violated and sum up the violation
    % otherwise i sum all the constraints that are satisfied
    for i=1:n_constraint
       % i sum only the constraints violation it means that i have to discard value less then zero 
       % in this part i consider only the violation on each constraint for the overall experiments
       index = constraints_violation(i,:)<=0;
       %% TODO check isempty on the other constraints manager
       if(prod(index))
           % the current constraints is always satisified
           %index = obj.constraints_violation(i,:)<=0;
           penalties(1,i) = sum(constraints_violation(i,index),2); 
       else
           % at least in one time step the current constraints is
           % not satisfied
           penalties(1,i) = sum(constraints_violation(i,~index),2); 
       end
    end    
    %% i check for all the constraints that are satisfied 
    % in index: 0 = not satisfied      1 = satisfied
    index = penalties <= 0; 
    violation_index = find(~index);
    %% because if the product is zero means that i have violation i sum only the violation 
    %% otherwise i sum all the satisfaction 
    if((prod(index,2)))
        y = sum(penalties,2);
    else
        penalties(index) = 0;
        y = sum(penalties,2);
    end
    
    y = y(1,:);
    
end