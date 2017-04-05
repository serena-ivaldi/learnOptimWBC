% i modified this function for taking into account also the regulation in the joint space
function [p ,pd,pdd,time]=Fixedpoint(parameters)

v = sym('v');
zero = sym('zero');

v_zero = zeros(1,length(parameters));

p   = v ;
pd  = zero;
pdd = zero;


p = subs(p,{v},{parameters'});
pd = subs(pd,{zero},{v_zero'});
pdd = subs(pdd,{zero},{v_zero'});

p = matlabFunction(p);
pd = matlabFunction(pd);
pdd = matlabFunction(pdd);

p = @(t)p();
pd = @(t)pd();
pdd = @(t)pdd();

time = 0;


end