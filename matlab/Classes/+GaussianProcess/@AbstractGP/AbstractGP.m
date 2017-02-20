classdef AbstractGP < handle
    
   properties(Abstract)
      X
      Y
   end
       
    
   methods(Abstract = true)
      Init(obj,X_i,Y_i); 
      Train(obj);
      Update(obj,x_n,y_n,train);
      y_t = Predict(obj,x_t);
   end
   
   methods
       function plot2D(obj,bounds)
       end
       
   end
    
end