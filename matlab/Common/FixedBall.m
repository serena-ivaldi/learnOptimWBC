% 
function [p ,pd,pdd,time]=FixedBall(parameters)

mean = sym('mean',[3,1]);
variance = sym('variance',[3,1]);
zero = sym('zero');

v_zero = zeros(1,length(parameters));

p   = mean + variance;
pd  = zero;
pdd = zero;


p = subs(p,{mean(1),mean(2),mean(3)},{parameters(1),parameters(2),parameters(3)});
pd = subs(pd,{zero},{v_zero'});
pdd = subs(pdd,{zero},{v_zero'});

p = matlabFunction(p,'vars',{variance});
pd = matlabFunction(pd);
pdd = matlabFunction(pdd);

p = @(t,variance)p(variance);
pd = @(t,variance)pd();
pdd = @(t,variance)pdd();

time = 0;


end