% inequality 
function violation = CheckBalance(zmp,support_poly)
    violation_flag = false;
    
   if(zmp(1)<support_poly.min(1) || zmp(1)<support_poly.max(1))
       violation_flag = true;
   end
   if(zmp(2)<support_poly.min(2) || zmp(2)<support_poly.max(2))
       violation_flag = true;
   end
   
    if(violation_flag)
         % here the value is positive because im in violation
         cx = max(min(zmp(1), support_poly.min(1)+support_poly.height), support_poly.min(1));
         cy = max(min(zmp(2), support_poly.min(2)+support_poly.height), support_poly.min(2));
         violation = sqrt( (zmp(1)-cx)*(zmp(1)-cx) + (zmp(2)-cy)*(zmp(2)-cy) );
    else
         % here the value si negative because the constraint sisatisfied
         violation = - norm(zmp - support_poly.center);
    end    
end