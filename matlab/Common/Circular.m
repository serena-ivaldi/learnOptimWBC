% rad_val      = geom_parameters(1);
% phi_val      = geom_parameters(2);
% theta_val    = geom_parameters(3);
% wzero_val    = geom_parameters(4);
% x_centre_val = geom_parameters(5);
% y_centre_val = geom_parameters(6);
% z_centre_val = geom_parameters(7);



function [p,pd,pdd,time] = Circular(s,time_struct,geom_parameters,type)
    
   t = sym('t');
   rad      = sym('rad');
   phi      = sym('phi');
   theta    = sym('theta');
   wzero    = sym('wzero');
   x_centre = sym('x_centre'); 
   y_centre = sym('y_centre');
   z_centre = sym('z_centre');

   %structure of the trajectory
   u = [-sin(theta);cos(theta);0];
   nxu =[cos(theta)*cos(phi);cos(theta)*sin(phi);-sin(theta)];
   centre =[x_centre;y_centre;z_centre];
   %parametric expression of the trajectory
   p   = rad*cos(2*pi*s + wzero)*u + rad*sin(2*pi*s + wzero)*nxu  + centre;
   pd  = diff(p,t);
   pdd = diff(pd,t);
   %assing numeric value to the parameter of the trajectory
   rad_val      = geom_parameters(1);
   phi_val      = geom_parameters(2);
   theta_val    = geom_parameters(3);
   wzero_val    = geom_parameters(4);
   x_centre_val = geom_parameters(5);
   y_centre_val = geom_parameters(6);
   z_centre_val = geom_parameters(7);
   
   p = subs(p,{rad,phi,theta,wzero,x_centre,y_centre,z_centre},{rad_val,phi_val,theta_val,wzero_val,x_centre_val,y_centre_val,z_centre_val});
   pd = subs(pd,{rad,phi,theta,wzero},{rad_val,phi_val,theta_val,wzero_val});
   pdd = subs(pdd,{rad,phi,theta,wzero},{rad_val,phi_val,theta_val,wzero_val});
   % transform the exrepssion in matlab function of t 
   p = matlabFunction(p);
   pd = matlabFunction(pd);
   pdd = matlabFunction(pdd);

   time = 0;
   
   if(strcmp(type,'sampled'))
      
      
      time=time_struct.ti:time_struct.step:time_struct.tf;
      %normtime = NormalizeTime(time,time_struct.ti,time_struct.tf);
 
      p = p(time);
      pd = pd(time);
      pdd = pdd(time);
    
      
   end
   
   
   
    
   
 
end
