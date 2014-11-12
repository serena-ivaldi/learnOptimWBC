%  x_centre = parameters(1);
%  y_centre = parameters(2);
%  z_centre = parameters(3);
%  rad      = parameters(4);
%  phi      = parameters(5);
%  theta    = parameters(6);
%  w        = parameters(7);
%  w0       = parameters(8);


function [p,pd,pdd,time] = CircularSymbolic(s,ti,tf,step,parameters,type)
    
    t = sym('t');
    x_centre = sym('x_centre'); 
    y_centre = sym('y_centre');
    z_centre = sym('z_centre');
    rad      = sym('rad');
    phi      = sym('phi');
    theta    = sym('theta');
    w        = sym('w');
    w0       = sym('w0');





   
   u = [-sin(theta);cos(theta);0];
   nxu =[cos(theta)*cos(phi);cos(theta)*sin(phi);-sin(theta)];
   centre =[x_centre;y_centre;z_centre];

   p   = rad*cos(2*pi*s + w0)*u + rad*sin(2*pi*s + w0)*nxu  + centre;
   pd  = diff(p,t);
   pdd = diff(pd,t);
   
   p = matlabFunction(p);
   pd = matlabFunction(pd);
   pdd = matlabFunction(pdd);
   time = 0;
   
   if(strcmp(type,'sampled'))
      
      x_centre = parameters(1);
      y_centre = parameters(2);
      z_centre = parameters(3);
      rad      = parameters(4);
      phi      = parameters(5);
      theta    = parameters(6);
      w        = parameters(7);
      w0       = parameters(8);
      
      time=ti:step:tf;
      normtime = NormalizeTime(time,ti,tf);
 
      p = p(normtime,phi,rad,theta,w0,x_centre,y_centre,z_centre);
      pd = pd(normtime,phi,rad,theta,w0);
      pdd = pdd(normtime,phi,rad,theta,w0);
      
   end
   
   
   
    
   
 
end
