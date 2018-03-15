function [r,p,y] = rpyFromRotation(R)
    r = atan2(R(3,2),R(3,3));
    p = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    y = atan2(R(2,1),R(1,1));
end