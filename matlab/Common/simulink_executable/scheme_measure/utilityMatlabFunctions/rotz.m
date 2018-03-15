function R = rotz(alpha)
   R = zeros(3, 3);
   R(3,3) =  1;
   R(1,1) =  cos(alpha);
   R(1,2) = -sin(alpha);
   R(2,1) =  sin(alpha);
   R(2,2) =  cos(alpha); 
end
