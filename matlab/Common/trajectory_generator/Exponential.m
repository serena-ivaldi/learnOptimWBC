

function [s]=Exponential(tf)

   a = 1; 
   t = sym('t');
   s(t) = (a*t^2)/(a*tf^2);
   
end