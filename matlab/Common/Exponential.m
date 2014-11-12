

function [s]=Exponential(tf,time_parameters)

   a = time_parameters(1); 
   t = sym('t');
   s(t) = (a*t^2)/(a*tf^2);
   
end