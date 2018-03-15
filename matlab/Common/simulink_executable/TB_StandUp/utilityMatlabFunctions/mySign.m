function [ sgn ] = mySign( a )
    if (a == 0); sgn = 1; return; end
    sgn = sign(a);
end

