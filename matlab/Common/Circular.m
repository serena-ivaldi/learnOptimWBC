% parameters[1] = height of the trajectory
% parameters[2] = x centre of trajectory 
% parameters[3] = y centre of trajectory
% parameters[4] = radius
% parameters[5] = omega
% parameters[6] = start_angle


function [p,pd,pdd] = Circular(t,parameters)
    
    height = parameters(1);
    x_centre = parameters(2);
    y_centre = parameters(3);
    rad      = parameters(4);
    w        = parameters(5);
    start_angle = parameters(6);

    % compute points on circumference
    x = rad*cos(w*t + start_angle) + x_centre;
    y = rad*sin(w*t + start_angle) + y_centre;
    z = height;
    
    xd = -rad*w*sin(w*t + start_angle);
    yd = rad*w*cos(w*t + start_angle);
    zd = 0;
    
    xdd = -rad*w^2*cos(w*t + start_angle);
    ydd = -rad*w^2*sin(w*t + start_angle);
    zdd = 0;
    
    % add extra row if z-coordinate is specified, but circle is always in xy plane
    p = [x y z];
    pd = [xd yd zd];
    pdd = [xdd ydd zdd];
 
end