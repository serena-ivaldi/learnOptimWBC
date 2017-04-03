% inequality of the kind  x < k  ---->  x - k < 0



function violation = CheckBalance(zmp,support_poly)
    violation_flag = false;
    
   if(zmp(1)<support_poly.min(1) || zmp(1)<support_poly.max(1))
       violation_flag = true;
   end
   if(zmp(2)<support_poly.min(2) || zmp(2)<support_poly.max(2))
       violation_flag = true;
   end
   
    if(violation_flag)
        violation = norm()
    else
        violation = - norm(zmp - support_poly.center);
    end    
end