% i modified this function for taking into account also the regulation in the joint space
function [p pd pdd time]=Fixedpoint(parameters)

p = parameters';
pd = zeros(size(parameters,2),1);
pdd = zeros(size(parameters,2),1);
time = -1;

end