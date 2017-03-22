% xi = geom_parameters(1);
% yi = geom_parameters(2);
% zi = geom_parameters(3);
% xf = geom_parameters(4);
% yf = geom_parameters(5);
% zf = geom_parameters(6);


function [p,pd,pdd,time] = Rectilinear(s,time_struct,geom_parameters,type)

    t = sym('t');
    xi = sym('xi');
    yi = sym('yi');
    zi = sym('zi');

    xf = sym('xf');
    yf = sym('yf');
    zf = sym('zf');

    pi = [xi;yi;zi];
    pf = [xf;yf;zf];

    p = pi + s*(pf-pi)/sqrt( (pf(1)-pi(1))^2 + (pf(2)-pi(2))^2 + (pf(3)-pi(3))^2 );
    pd = diff(p,t);
    pdd = s*zeros(3,1);

    xi_val = geom_parameters(1);
    yi_val = geom_parameters(2);
    zi_val = geom_parameters(3);

    xf_val = geom_parameters(4);
    yf_val = geom_parameters(5);
    zf_val = geom_parameters(6);


    p = subs(p,{xi,yi,zi,xf,yf,zf},{xi_val,yi_val,zi_val,xf_val,yf_val,zf_val});
    pd = subs(pd,{xi,yi,zi,xf,yf,zf},{xi_val,yi_val,zi_val,xf_val,yf_val,zf_val});
   
    
      % check if the some of the function are constant
      check1=symvar(p);
      if(isempty(check1))
         p(4) = t;
      end
      check2=symvar(pd);
      if(isempty(check2))
         pd(4)= t;
      end
      check3=symvar(pdd);
      if(isempty(check3))
         pdd(4) = t;
      end
    
    % transform the expression in matlab function of t 
    p = matlabFunction(p);
    pd = matlabFunction(pd);
    pdd = matlabFunction(pdd);

    time = 0;
    
    if(strcmp(type,'sampled'))
      
      
      time=time_struct.ti:time_struct.step:time_struct.tf;
 
      p = p(time);
      pd = pd(time);
      pdd = pdd(time);
    
      
   end
   
    
end