% input is a structure contains the two point whom we want to mesure the
% distance
% constraints_values is  a structure contains the distance we want ot keep and the tolerance

function violation = FixedDistanceEquality(input,constraints_values)

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

