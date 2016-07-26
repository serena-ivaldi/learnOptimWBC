function violation = FixedDistanceEquality(input,constraints_values)
%% FixedDistanceEquality
%   A constraints of distance between to point under a specify tolerance
% Arguments :
%              input - Structure containing the two point whom we want to
%                      mesure the distance, the tolerance of the constraint
%                      and the cativatio flag of the constraint
% constraints_values - The distance we want to keep

    dist = norm(input.p1 - input.p2);
    diff = abs(dist - constraints_values);
    if input.active
        if diff < input.epsilon
            violation = 0;
        else
            violation = diff  - input.epsilon;
        end
    else
        violation = 0;
    end
end

