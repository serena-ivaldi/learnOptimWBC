%% this function can be used to overcome the restriction that arises with the function to compute constraints
%% for this kind of function you can provide only one input and one constraints_values value.
%% sometimes you need much more information to copmpute the constraint value.
%% so to overcome this issue with EmptyConstraints you can compute the constraints violation directly in the fitness function
%% and pass the results using the input variable

function violation = EmptyConstraints(input,constraints_values)
    
    violation = input;
    
end