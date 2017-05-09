% n_of_basis   = parameters(1);
% redundancy   = parameters(2);
% range        = parameters(3); 


function [p,pd,pdd,time,obj4visual] = AdHocBalanceControllerTrajectoryTwoTimeLaws(s_ext,time_struct,parameters,type)
   
   n_of_basis_time_law    = parameters(1);
   n_of_basis_geometric   = parameters(2);
   redundancy             = parameters(3);
   starting_position_x    = parameters(4);  
   starting_position_y    = parameters(5);
   starting_position_z    = parameters(6); 
   ending_position_x      = parameters(7);  
   ending_position_y      = parameters(8);
   ending_position_z      = parameters(9);
   
   
   % i need to more basis to fix the first and the last value of the rbf
   real_n_of_basis_time_law  = n_of_basis_time_law + 2;
   real_n_of_basis_geometric = n_of_basis_geometric + 2;
   
   t = sym('t');
   v = sym('v');
   theta = sym('theta',[n_of_basis_time_law * 2  + n_of_basis_geometric*2,1]);
   
   
   %% time law x
   T = time_struct.tf;
   sigma = (T)/((real_n_of_basis_time_law-1)*redundancy); % this value of sigma produces a 15% of overlapping between two consecutive gaussian with redundancy = 3;
   cof = 2*sigma^2;
   sumphi = 0;
   phi = sym('phi', [1,real_n_of_basis_time_law]);
   % first and last basis function are special
   for i=0:(real_n_of_basis_time_law - 1)
       phi(i + 1) = exp(-(t-(i*T)/(real_n_of_basis_time_law-1))^2/cof);
       basis_functions_time{i + 1} = matlabFunction(phi(i+1));
       sumphi = sumphi + phi(i + 1);
   end
   
   
   
   %% plot basis functions
%    time = time_struct.ti:time_struct.step:time_struct.tf;
%             
%    for j=1:real_n_of_basis
%        i=1;
%        for ti = time
%            results(j,i) = feval(basis_functions_time{j},ti); 
%            i=i+1;
%        end
%    end    
%    figure
%    hold all;
%    for j=1:real_n_of_basis
%        plot(results(j,:))
%    end
   %% end plot
   
   
   mat = [subs(phi(1), t, 0)      subs(phi(end), t, 0);
          subs(phi(1), t, T)      subs(phi(end), t, T) ];
      
   phi_computed_in_start =[];
   phi_computed_in_end   =[];
   for i=1:length(phi)
       phi_computed_in_start = [phi_computed_in_start,subs(phi(i), t, 0)];
       phi_computed_in_end   = [phi_computed_in_end  ,subs(phi(i), t, T)];
   end   
         
   y0 = 0;
   yf = 1;
   vector = [y0*sum(phi_computed_in_start) - phi_computed_in_start(2:end-1)*theta(1:n_of_basis_time_law); 
             yf*sum(phi_computed_in_end)   - phi_computed_in_end(2:end-1)*theta(1:n_of_basis_time_law) ];
         
   lambda = mat\vector;
   
   s_x = (phi(2:end-1)*theta(1:n_of_basis_time_law) + lambda(1)*phi(1) + lambda(2)*phi(end))/sumphi;
   
    %% time law z
   T = time_struct.tf;
   sigma = (T)/((real_n_of_basis_time_law-1)*redundancy); % this value of sigma produces a 15% of overlapping between two consecutive gaussian with redundancy = 3;
   cof = 2*sigma^2;
   sumphi = 0;
   phi = sym('phi', [1,real_n_of_basis_time_law]);
   % first and last basis function are special
   for i=0:(real_n_of_basis_time_law - 1)
       phi(i + 1) = exp(-(t-(i*T)/(real_n_of_basis_time_law-1))^2/cof);
       basis_functions_time{i + 1} = matlabFunction(phi(i+1));
       sumphi = sumphi + phi(i + 1);
   end
   
    
   mat = [subs(phi(1), t, 0)      subs(phi(end), t, 0);
          subs(phi(1), t, T)      subs(phi(end), t, T) ];
      
   phi_computed_in_start =[];
   phi_computed_in_end   =[];
   for i=1:length(phi)
       phi_computed_in_start = [phi_computed_in_start,subs(phi(i), t, 0)];
       phi_computed_in_end   = [phi_computed_in_end  ,subs(phi(i), t, T)];
   end   
         
   y0 = 0;
   yf = 1;
   vector = [y0*sum(phi_computed_in_start) - phi_computed_in_start(2:end-1)*theta(n_of_basis_time_law + 1:n_of_basis_time_law*2); 
             yf*sum(phi_computed_in_end)   - phi_computed_in_end(2:end-1)*theta(n_of_basis_time_law + 1:n_of_basis_time_law*2) ];
         
   lambda = mat\vector;
   
   s_z = (phi(2:end-1)*theta(n_of_basis_time_law + 1:n_of_basis_time_law*2) + lambda(1)*phi(1) + lambda(2)*phi(end))/sumphi;
   
   
   
   
   %% geometric path
   T = 1;
   sigma = (T)/((real_n_of_basis_geometric-1)*redundancy); % this value of sigma produces a 15% of overlapping between two consecutive gaussian with redundancy = 3;
   cof = 2*sigma^2;
   sumphi = 0;
   phi = sym('phi', [1,real_n_of_basis_geometric]);
   for i=0:(real_n_of_basis_geometric - 1)
       phi(i + 1) = exp(-(v-(i*T)/(real_n_of_basis_geometric-1))^2/cof);
       basis_functions_time{i + 1} = matlabFunction(phi(i+1));
       sumphi = sumphi + phi(i + 1);
   end
   
      %% plot basis functions
%    time = time_struct.ti:0.001:T;
%             
%    for j=1:real_n_of_basis
%        i=1;
%        for ti = time
%            results(j,i) = feval(basis_functions_time{j},ti); 
%            i=i+1;
%        end
%    end    
%    figure
%    hold all;
%    for j=1:real_n_of_basis
%        plot(results(j,:))
%    end
   %% end plot
   
   mat = [subs(phi(1), v, 0)      subs(phi(end), v, 0);
          subs(phi(1), v, T)      subs(phi(end), v, T) ];
      
   phi_computed_in_start =[];
   phi_computed_in_end   =[];
   for i=1:length(phi)
       phi_computed_in_start = [phi_computed_in_start,subs(phi(i), v, 0)];
       phi_computed_in_end   = [phi_computed_in_end  ,subs(phi(i), v, T)];
   end   
   
   o_x = starting_position_x;
   f_x = ending_position_x;
   vector = [o_x*sum(phi_computed_in_start) - phi_computed_in_start(2:end-1) * theta(n_of_basis_time_law*2 + 1: n_of_basis_time_law*2 + n_of_basis_geometric ); 
             f_x*sum(phi_computed_in_end)   - phi_computed_in_end(2:end-1)   * theta(n_of_basis_time_law*2 + 1: n_of_basis_time_law*2 + n_of_basis_geometric ) ];    
   lambda = mat\vector;      
   
   rbfx = ( phi(2:end-1)*theta(n_of_basis_time_law*2 + 1:n_of_basis_time_law*2 + n_of_basis_geometric) + lambda(1)*phi(1) + lambda(2)*phi(end) )/sumphi;
  
   p_visual   = rbfx;
  
   rbfx = subs(rbfx,v,s_x);
   
   rbfy = starting_position_y;
   
   p_visual   = [p_visual;rbfy];
  
   o_z = starting_position_z;
   f_z = ending_position_z;
   vector = [o_z*sum(phi_computed_in_start) - phi_computed_in_start(2:end-1) * theta(n_of_basis_time_law*2 + n_of_basis_geometric + 1: n_of_basis_time_law*2 + n_of_basis_geometric*2); 
             f_z*sum(phi_computed_in_end)   - phi_computed_in_end(2:end-1)   * theta(n_of_basis_time_law*2 + n_of_basis_geometric + 1: n_of_basis_time_law*2 + n_of_basis_geometric*2) ];
   lambda = mat\vector; 
   
   rbfz = + ( phi(2:end-1)*theta(n_of_basis_time_law*2 + n_of_basis_geometric + 1:n_of_basis_time_law*2 + n_of_basis_geometric*2)  + lambda(1)*phi(1) + lambda(2)*phi(end) )/sumphi;
   
   p_visual   = [p_visual;rbfz];
  
   rbfz = subs(rbfz,v,s_z);
   
   p   = [rbfx;rbfy;rbfz];
   
   % i need this to change the value from time to s coordinate to add the
   % time law
   %p = subs(p,v,s);
   
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
   %% for visualization and for execution (TODO change the name fo the struct in a more meaningfull way)
   
   
   p_visual   = matlabFunction(p_visual,'vars', {v,theta});
   
   obj4visual.p_real = p; 
   obj4visual.p_test = p_visual;
   sd_x = diff(s_x,t);
   sdd_x = diff(sd_x,t);
   
   s_x   = matlabFunction(s_x,'vars', {t,theta});
   sd_x  = matlabFunction(sd_x,'vars', {t,theta});
   sdd_x = matlabFunction(sdd_x,'vars', {t,theta});
   
   obj4visual.s_x = s_x;
   obj4visual.sd_x = sd_x;
   obj4visual.sdd_x = sdd_x;
   
   sd_z = diff(s_z,t);
   sdd_z = diff(sd_z,t);
   
   s_z   = matlabFunction(s_z,'vars', {t,theta});
   sd_z  = matlabFunction(sd_z,'vars', {t,theta});
   sdd_z = matlabFunction(sdd_z,'vars', {t,theta});
   
   obj4visual.s_z = s_z;
   obj4visual.sd_z = sd_z;
   obj4visual.sdd_z = sdd_z;
   
   
   
end
