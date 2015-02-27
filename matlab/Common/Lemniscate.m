% rad_val      = geom_parameters(1);
% phi_val      = geom_parameters(2);
% theta_val    = geom_parameters(3);
% wzero_val    = geom_parameters(4);
% x_centre_val = geom_parameters(5);
% y_centre_val = geom_parameters(6);
% z_centre_val = geom_parameters(7);



function [p,pd,pdd,time] = Lemniscate(s,time_struct,geom_parameters,type)
 
   t = sym('t');
   a        = sym('a');
   tau      = sym('tau');
   alpha    = sym('alpha');
   x_centre = sym('x_centre'); 
   y_centre = sym('y_centre');
   z_centre = sym('z_centre');

   T = roty(alpha);
   R = T(1:3,1:3);
   centre = [x_centre ; y_centre; z_centre];
   %parametric expression of the trajectory
   x = (a*cos(s + tau)/(1+sin(s + tau)^2));
   y = (a*sin(s + tau)*cos(s + tau)/(1+sin(s + tau)^2));
   z = 0;
   
   p = [x;y;z];
   p = R*p + centre;
   
   pd  = diff(p,t);
   pdd = diff(pd,t);
   %assing numeric value to the parameter of the trajectory
   a_val        = geom_parameters(1);
   alpha_val    = geom_parameters(2);
   tau_val      = geom_parameters(3);
   x_centre_val = geom_parameters(4);
   y_centre_val = geom_parameters(5);
   z_centre_val = geom_parameters(6);
   
   p = subs(p,{a,alpha,tau,x_centre,y_centre,z_centre},{a_val,alpha_val,tau_val,x_centre_val,y_centre_val,z_centre_val});
   pd = subs(pd,{a,alpha,tau},{a_val,alpha_val,tau_val});
   pdd = subs(pdd,{a,alpha,tau},{a_val,alpha_val,tau_val});
   % transform the expression in matlab function of t 
   p = matlabFunction(p);
   pd = matlabFunction(pd);
   pdd = matlabFunction(pdd);

   time = 0;
   
   if(strcmp(type,'sampled'))
      
      
      time=time_struct.ti:time_struct.step:time_struct.tf;
 
      p = p(time);
      pd = pd(time);
      pdd = pdd(time);
    
      
   end
   
   
   
    
   
 
end
