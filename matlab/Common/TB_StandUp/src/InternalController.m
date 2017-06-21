function tau = InternalController(q_des,q,qd,M,biasTorque)

    Ndof = 23;

    p = 1;
    d = 2*sqrt(p);
    Kp = eye(Ndof)*p;
    Kd = eye(Ndof)*d;


    M_red = M (7:end,7:end);
    bias_red = biasTorque(7:end);

    q_err = sum(q_des - q,1);

    %tau = M_red*(Kd*(-qd) + Kp*(q_des - q)) + bias_red;
     tau =  Kp*(q_des - q) + Kd*(-qd);

end



