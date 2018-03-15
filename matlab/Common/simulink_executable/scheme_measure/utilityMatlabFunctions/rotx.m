function R = rotx(alpha)
   R = zeros(3, 3);
   R(1,1) =  1;
   R(2,2) =  cos(alpha);
   R(2,3) = -sin(alpha);
   R(3,2) =  sin(alpha);
   R(3,3) =  cos(alpha); 
end
