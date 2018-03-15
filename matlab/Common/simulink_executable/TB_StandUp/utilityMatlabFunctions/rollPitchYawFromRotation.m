 %#codegen
function rollPitchYaw = rollPitchYawFromRotation(R)
% From http://www.geometrictools.com/Documentation/EulerAngles.pdf
rollPitchYaw = zeros(3,1);
if (R(3,1) < +1) 
    if (R(3,1) > -1) 
        rollPitchYaw(2) = asin(-R(3,1)); 
        rollPitchYaw(3) = atan2(R(2,1),R(1,1)); 
        rollPitchYaw(1) = atan2(R(3,2), R(3,3));
    else
        rollPitchYaw(3) =-atan2(-R(2,3),R(2,2));
        rollPitchYaw(1) = 0;
    end
else
    rollPitchYaw(3) = atan2(-R(2,3),R(2,2));
    rollPitchYaw(1) = 0;
end