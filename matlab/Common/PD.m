function control=PD(x,x_des,Kp,xd,xd_des,Kd,xdd_des)

control = xdd_des + Kd*(xd_des - xd) + Kp*(x_des - x);

%% DEBUG
errorx = norm(x_des - x)
errorxd= norm(xd_des - xd)
%%---
end