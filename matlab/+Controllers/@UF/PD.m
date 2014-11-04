function  F=PD(x,x_des,Kp,xd,xd_des,Kd,xdd_des)

    F = xdd_des - Kd*(xd_des - xd) - Kp(x_des - x);

end