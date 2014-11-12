


function [p,pd,pdd] = CircularTest(time_struct,type,parameters)
    
   
    rad      = parameters(1);
    phi      = parameters(2);
    theta    = parameters(3);
    w        = parameters(4);
    w0       = parameters(5);
    x_centre = parameters(6);
    y_centre = parameters(7);
    z_centre = parameters(8);
    
    p =[];
    pd = [];
    pdd = [];

%     compute points on circumference
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


if(strcmp(type,'func'))
   u = [-sin(theta);cos(theta);0];
   nxu =[cos(theta)*cos(phi);cos(theta)*sin(phi);-sin(theta)];
   centre =[x_centre;y_centre;z_centre];

   p= rad*cos(w*t + w0)*u + rad*sin(w*t + w0)*nxu  + centre;
   pd = -w*rad*sin(w*t + w0)*u + w*rad*cos(w*t + w0)*nxu;
   pdd = -w^2*rad*cos(w*t + w0)*u-w^2*rad*sin(w*t + w0)*nxu;

    
elseif(strcmp(type,'sampled'))
   u = [-sin(theta);cos(theta);0];
   nxu =[cos(theta)*cos(phi);cos(theta)*sin(phi);-sin(theta)];
   centre =[x_centre;y_centre;z_centre];

  
   for t=time_struct.ti:time_struct.step:time_struct.tf;
       
       t = NormalizeTime(t,time_struct.ti,time_struct.tf);
       
       p_cur= rad*cos(w*t + w0)*u + rad*sin(w*t + w0)*nxu  + centre;
       pd_cur = -w*rad*sin(w*t + w0)*u + w*rad*cos(w*t + w0)*nxu;
       pdd_cur = -w^2*rad*cos(w*t + w0)*u-w^2*rad*sin(w*t + w0)*nxu;
       
       p= [p,p_cur];
       pd = [pd,pd_cur];
       pdd = [pdd,pdd_cur];
       
   end

end
    
   
 
end
