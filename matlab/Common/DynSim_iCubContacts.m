function [tauContact]=DynSim_iCubContacts(ndof,state,dynamic,contact_jacobian,param,tau)
    import WBM.utilities.frame2posRotm;
    import WBM.utilities.rotm2eulAngVelTF
    Jc = contact_jacobian.Jc;
    Jc_t = Jc';
    dJcNu = contact_jacobian.dJcNu;
    M = dynamic.M;
    h = dynamic.h;
    q = state.q;
    w_R_b = state.w_R_b;                      
    x_b = state.x_b;
    Nu = state.Nu;
    
    %% moving floating base
    % feet correction gain
    K_corr_pos  = 2.5;
    K_corr_vel  = 2*sqrt(K_corr_pos);
    % feet current position and orientation
    l_sole   = wbm_forwardKinematics(w_R_b,x_b,q,'l_sole');
    r_sole   = wbm_forwardKinematics(w_R_b,x_b,q,'r_sole');
    [x_lfoot,R_b_lfoot]    = frame2posRotm(l_sole);
    [x_rfoot,R_b_rfoot]    = frame2posRotm(r_sole);

    % orientation is parametrized with euler angles
    [phi_lfoot,~]          = rotm2eulAngVelTF(R_b_lfoot);
    [phi_rfoot,~]          = rotm2eulAngVelTF(R_b_rfoot);

    pos_leftFoot           = [x_lfoot; phi_lfoot];
    pos_rightFoot          = [x_rfoot; phi_rfoot];

    % feet original position and orientation
    lsole_ini              = param.lfoot_ini;
    rsole_ini              = param.rfoot_ini;

    [xi_lfoot,R_bi_lfoot]  = frame2posRotm(lsole_ini);
    [xi_rfoot,R_bi_rfoot]  = frame2posRotm(rsole_ini);

    [phi_rfoot_ini,~]      = rotm2eulAngVelTF(R_bi_rfoot);
    [phi_lfoot_ini,~]      = rotm2eulAngVelTF(R_bi_lfoot);

    lfoot_ini_tot          = [xi_lfoot; phi_lfoot_ini];
    rfoot_ini_tot          = [xi_rfoot; phi_rfoot_ini];

    % error between original and current feet position and orientation
    if param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 0

        pos_feet_delta = pos_leftFoot-lfoot_ini_tot;

    elseif param.feet_on_ground(1) == 0 && param.feet_on_ground(2) == 1

        pos_feet_delta = pos_rightFoot-rfoot_ini_tot;

    elseif param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 1

        pos_feet_delta = [(pos_leftFoot-lfoot_ini_tot);...
            (pos_rightFoot-rfoot_ini_tot)];
    end
    %% Compute contact forces
    % % Real contact forces computation
    S               = [ zeros(6,ndof); eye(ndof,ndof)];
    JcMinv          = Jc/M;
    JcMinvS         = JcMinv*S;
    % the way which fc is calculated has to be checked
    fc              = (JcMinv*Jc_t)\(JcMinv*h -JcMinvS*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*pos_feet_delta);
    
    tauContact = Jc_t*fc;
    
    
end