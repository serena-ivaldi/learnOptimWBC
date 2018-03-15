function R = rotationFromAxisAngle(q)
    %Rodriques's formula
    % R = I3 + sin(theta) S(r) + 2 sin(theta/2)^2 S(r)^2,
    u     = q(1:3);
    theta = q(4); 
    R = eye(3) +  sin(theta) * Sf(u) + 2 * sin(theta/2)^2 * Sf(u)^2;
end
