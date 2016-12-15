classdef GPML_GP < GaussianProcess.AbstractGP
    
   properties
      X
      Y
      likelihood
      inference
      cov 
      mean
      hyp0
      hyp
      Ncg
   end
       
    
   methods
       %% TODO for now I setup everything inside the constructor then I will
       % do it in a flexible way
       function obj = GPML_GP()
           obj.likelihood = 'likGauss';
           sn = 0.2;
           obj.hyp0.lik  = log(sn);
           obj.inference = 'infGaussLik';
           obj.cov = {@covSEiso}; 
           ell = 1/4; sf = 1;                       
           obj.hyp0.cov  = log([ell;sf]);
           obj.mean = {@meanZero};
           obj.hyp0.mean = [];
           obj.Ncg = 50;
       end
       
       function Init(obj,X_i,Y_i)
           obj.X = X_i;
           obj.Y = Y_i;
           if obj.Ncg == 0
             obj.hyp = obj.hyp0;
           else
             obj.hyp = minimize(obj.hyp0,'gp', -obj.Ncg, obj.inference, obj.mean, obj.cov, obj.likelihood, obj.X, obj.Y); % opt hypers
           end
       end
       
       %% TODO understand if it is usefull
       function Train(obj,x_i,y_i)
           
       end
       
       function Update(obj,x_n,y_n) 
           obj.X(end + 1,:) = x_n;
           obj.Y(end + 1) = y_n;
           if obj.Ncg == 0
             obj.hyp = obj.hyp0;
           else
             obj.hyp = minimize(obj.hyp,'gp', -obj.Ncg, obj.inference, obj.mean, obj.cov, obj.likelihood, obj.X, obj.Y); % opt hypers
           end
       end
       
       function [ymu,ys2] = Predict(obj,x_t)
           [ymu, ys2, fmu, fs2 ] = gp(obj.hyp, obj.inference, obj.mean, obj.cov, obj.likelihood, obj.X, obj.Y, x_t); 
       end
       
   end
    
end