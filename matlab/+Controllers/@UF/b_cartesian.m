function  F=b_cartesian(obj,x_cur,x_des,Kp,xd_cur,xd_des,Kd,xdd_des,q)


F = xdd_des - Kd*(xd_des - xd_cur) - Kp(x_des - x_cur) - obj.Jd_vec*q;

end