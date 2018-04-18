% inequality constraint
function violation = CheckBalance(zmp,support_poly)
% inputs: 
% zmp, coordinates of the ZMP, (2 x 2) matrix
% support_poly, variable containing information related to the support
% polygon, as defined in the iCub class
% 
% output:
% if the ZMP is outside of the support polygon,
%    violation is the distance between the ZMP and the closest support polygon boundary
% otherwise
%    violation is a negative number, a measure of how far the ZMP is from support polygon boundaries

   violation_flag = false;
   
   % Check if the ZMP was outside of the support polygon
   if(zmp(1)<support_poly.min(1) || zmp(1)>support_poly.max(1))
       violation_flag = true;
   end
   if(zmp(2)<support_poly.min(2) || zmp(2)>support_poly.max(2))
       violation_flag = true;
   end
   
   if(violation_flag)
        % here the value is positive because the constraint is in violation
        
        % if out of support polygon, output the closest support polygon boundary;
        % otherwise,                 output the ZMP coordinate
        cx = max(min(zmp(1), support_poly.max(1)), support_poly.min(1));
        cy = max(min(zmp(2), support_poly.max(2)), support_poly.min(2));
        
        %distance between the ZMP and the closest support polygon boundary
        violation = sqrt( (zmp(1)-cx)*(zmp(1)-cx) + (zmp(2)-cy)*(zmp(2)-cy) );
   
   else
        % here the value is negative because the constraint is satisfied     
        % violation is a measure of how far the ZMP is from support polygon boundaries
        % the lower violation is, the farther
        violation = -( support_poly.max_dist - norm(zmp - support_poly.center));
   end    
end