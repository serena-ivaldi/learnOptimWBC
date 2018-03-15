function q = quaternionFromEulerRotation(angle, axes)
    q      = zeros(4, 1);
    q(1)   = cos(angle/2);
    q(2:4) = sin(angle/2) .* axes;
end
