classdef AbstractGP < handle
    
   properties(Abstract)
      X
      Y
   end
       
    
   methods(Abstract = true)
      Init(X_i,Y_i); 
      Train(x_i,y_i);
      Update(x_n,y_n);
      y_t = Predict(x_t);
   end
   
   methods
       function plot2D(obj,bounds)
       end
       
   end
    
end