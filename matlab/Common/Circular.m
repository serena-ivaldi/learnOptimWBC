%  x_centre = parameters(1);
%  y_centre = parameters(2);
%  z_centre = parameters(3);
%  rad      = parameters(4);
%  phi      = parameters(5);
%  theta    = parameters(6);
%  w        = parameters(7);
%  w0       = parameters(8);


function [p,pd,pdd] = Circular(t,parameters)
    
    x_centre = parameters(1);
    y_centre = parameters(2);
    z_centre = parameters(3);
    rad      = parameters(4);
    phi      = parameters(5);
    theta    = parameters(6);
    w        = parameters(7);
    w0       = parameters(8);

    % compute points on circumference
%     x = rad*cos(w*t + start_angle) + x_centre;
%     y = rad*sin(w*t + start_angle) + y_centre;
%     z = height;
%     
%     xd = -rad*w*sin(w*t + start_angle);
%     yd = rad*w*cos(w*t + start_angle);
%     zd = 0;
%     
%     xdd = -rad*w^2*cos(w*t + start_angle);
%     ydd = -rad*w^2*sin(w*t + start_angle);
%     zdd = 0;


%   add extra row if z-coordinate is specified, but circle is always in xy plane
%     p = [x y z];
%     pd = [xd yd zd];
%     pdd = [xdd ydd zdd];


u = [-sin(theta);cos(theta);0];
nxu =[cos(theta)*cos(phi);cos(theta)*sin(phi);-sin(theta)];
centre =[x_centre;y_centre;z_centre];


p= rad*cos(w*t + w0)*u + rad*sin(w*t + w0)*nxu  + centre;
pd = -w*rad*sin(w*t + w0)*u + w*rad*cos(w*t + w0)*nxu;
pdd = -w^2*rad*cos(w*t + w0)*u-w^2*rad*sin(w*t + w0)*nxu;

p = p';
pd = pd';
pdd = pdd';


    
   
 
end
