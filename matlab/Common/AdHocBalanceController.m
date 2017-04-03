% n_of_basis   = parameters(1);
% redundancy   = parameters(2);
% range        = parameters(3); 


function [p,pd,pdd,time] = AdHocBalanceController(s,time_struct,parameters,type)
 
    n_of_basis   = parameters(1);
    redundancy   = parameters(2);
    starting_position = parameter(3);  
   % range        = parameters(3); 
   
   t = sym('t');
   theta = sym('theta',[n_of_basis*3,1]);
   T = time_struct.tf;

   % this value of sigma produces a 15% of overlapping between two 
   % consecutive gaussian with redundancy = 3;
   sigma = (T)/((n_of_basis-1)*redundancy);
   cof = 2*sigma^2;
   sumphi = 0;
   for i=0:(n_of_basis-1)
       phi(i+1) = exp(-(t-(i*T)/(n_of_basis-1))^2/cof);
       obj.basis_functions_time{i+1} = matlabFunction(phi(i+1));
       sumphi = sumphi + phi(i+1);
   end

   % trajecotory in time 
   s = (phi*theta(1:n_of_basis))/sumphi;
   
   for i=0:(n_of_basis-1)
       phi(i+1) = exp(-(s-(i*T)/(n_of_basis-1))^2/cof);
       obj.basis_functions{i+1} = matlabFunction(phi(i+1));
       sumphi = sumphi + phi(i+1);
   end
  
   % trajectory in space
   rbfx = starting_position(1) + (phi*theta(n_of_basis+1:n_of_basis*2))/sumphi;
   rbfy = starting_position(2);
   rbfz = starting_position(3) + (phi*theta(n_of_basis*2 + 1:n_of_basis*3))/sumphi;
   
   p   = [rbfx;rbfy;rbfz];
   pd  = diff(p,t);
   pdd = diff(pd,t);
   
  
   % transform the expression in matlab function of t 
   p = matlabFunction(p,'vars', {t,theta});
   pd = matlabFunction(pd,'vars', {t,theta});
   pdd = matlabFunction(pdd,'vars', {t,theta});

   time = 0;
   
   if(strcmp(type,'sampled'))
      
      
      time=time_struct.ti:time_struct.step:time_struct.tf;
 
      p = p(time);
      pd = pd(time);
      pdd = pdd(time);
    
      
   end
   
end
