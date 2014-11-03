function  F=b_joint(q_cur,q_des,Kp,qd_cur,qd_des,Kd,qdd_des)


F = qdd_des - Kd*(qd_des - qd_cur) - Kp(q_des - q_cur);

end